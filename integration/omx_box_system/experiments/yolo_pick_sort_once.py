#!/usr/bin/env python3
"""One-shot, guarded YOLO pick/sort sequence for OMX-F.

Start with the robot at home and the gripper open. The node takes a stable defect
target from /yolo/selected_box and performs the already validated calculation-
based pick/sort motion without interactive trajectory edits.
"""
import math
import sys
import time

import numpy as np
from scipy.optimize import least_squares

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener


ARM = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
# Same final pose as OMX-F bringup initial_positions.yaml step2.
HOME = [0.0, -1.57, 1.57, 1.57, 0.0]
SORT_HIGH_XY = (0.080, -0.200)
MAX_PITCH = 1.612214
TOOL = 0.12063


def ik(x, y, z_calc, q5, pitch=MAX_PITCH):
    q1 = math.atan2(y, x + 0.01125)
    radial = (x + 0.01125) * math.cos(q1) + y * math.sin(q1)

    def error(q):
        q2, q3 = q
        planar = (0.0415 * math.cos(q2) + 0.11315 * math.sin(q2)
                  + 0.162 * math.cos(q2 + q3) + TOOL * math.cos(pitch))
        z = (0.0975 - 0.0415 * math.sin(q2) + 0.11315 * math.cos(q2)
             - 0.162 * math.sin(q2 + q3) - TOOL * math.sin(pitch))
        return [planar - radial, z - z_calc]

    result = least_squares(error, [0.15, -0.3], bounds=([-2.0944, -2.0944], [1.5708, 1.5708]))
    q2, q3 = result.x
    q4 = pitch - q2 - q3
    q = [q1, q2, q3, q4, q5]
    if np.linalg.norm(result.fun) > 0.0015 or abs(q4) > 2.0944:
        raise RuntimeError(f'IK unreachable (residual={np.linalg.norm(result.fun):.4f})')
    return q


def select_pick_pitch(x, y, q5):
    """Use the most downward pitch that supports the full descent range."""
    for pitch in np.linspace(MAX_PITCH, 0.60, 80):
        try:
            # Checking both ends keeps one fixed pitch for above, descend and lift.
            ik(x, y, 0.115, q5, pitch)
            ik(x, y, 0.026, q5, pitch)
            ik(x, y, 0.045, q5, pitch)
            return float(pitch)
        except RuntimeError:
            pass
    raise RuntimeError('target unreachable even with adaptive pitch')


class Runner(Node):
    def __init__(self):
        super().__init__('yolo_pick_sort_once')
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        self.create_subscription(Float64MultiArray, '/yolo/selected_box', self.target_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.tf = Buffer(); self.tfl = TransformListener(self.tf, self)
        self.samples = []; self.joints = {}; self.gripper_pos = None

    def target_cb(self, msg):
        if len(msg.data) == 5 and msg.data[0] == 1.0 and msg.data[1] >= 0.35:
            values = np.asarray(msg.data[2:5], float)
            if np.isfinite(values).all():
                self.samples.append(values)
                self.samples = self.samples[-20:]

    def joint_cb(self, msg):
        self.joints.update(dict(zip(msg.name, msg.position)))
        self.gripper_pos = self.joints.get('gripper_joint_1')

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def z(self):
        try:
            return self.tf.lookup_transform('link0', 'end_effector_link', rclpy.time.Time()).transform.translation.z
        except Exception:
            return None

    def xyz(self):
        try:
            p = self.tf.lookup_transform('link0', 'end_effector_link', rclpy.time.Time()).transform.translation
            return p.x, p.y, p.z
        except Exception:
            return None

    def move(self, q, seconds, label):
        traj = JointTrajectory(); traj.joint_names = ARM
        point = JointTrajectoryPoint(); point.positions = list(map(float, q))
        whole = int(seconds); point.time_from_start = Duration(sec=whole, nanosec=int((seconds-whole)*1e9))
        traj.points = [point]; self.arm_pub.publish(traj)
        self.get_logger().info(f'{label}: {np.round(q,4).tolist()} ({seconds:.1f}s)')
        self.spin_for(seconds + 0.45)
        if not all(name in self.joints for name in ARM):
            raise RuntimeError('joint state unavailable')
        actual = np.array([self.joints[name] for name in ARM])
        if np.max(np.abs(actual - np.asarray(q))) > 0.13:
            raise RuntimeError(f'{label}: joint tracking failure')

    def grip(self, position, effort, expect_stall=False):
        if not self.gripper.wait_for_server(timeout_sec=3.0):
            raise RuntimeError('gripper action unavailable')
        goal = GripperCommand.Goal(); goal.command.position = position; goal.command.max_effort = effort
        future = self.gripper.send_goal_async(goal)
        while rclpy.ok() and not future.done(): rclpy.spin_once(self, timeout_sec=0.05)
        handle = future.result()
        if not handle.accepted: raise RuntimeError('gripper goal rejected')
        result_future = handle.get_result_async()
        deadline = time.monotonic() + 8.0
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not result_future.done():
            # A grasped object can keep the controller active while it holds effort.
            if expect_stall and self.gripper_pos is not None and 0.30 <= self.gripper_pos <= 0.43:
                self.get_logger().info(f'GRASP HELD at {self.gripper_pos:.3f} rad')
                return
            raise RuntimeError('gripper timeout')
        result = result_future.result().result
        if expect_stall and not (result.stalled and result.position <= 0.43):
            raise RuntimeError(f'grasp failed (position={result.position:.3f})')

    def run(self):
        self.spin_for(0.5)
        if '--resume-grasped' in sys.argv:
            pose = self.xyz()
            if pose is None or self.gripper_pos is None or not 0.30 <= self.gripper_pos <= 0.43:
                raise RuntimeError('resume requested but no grasp is detected')
            x, y, z = pose; q5 = self.joints['joint5']
            if not 0.020 <= z <= 0.040:
                raise RuntimeError(f'resume grasp Z is unsafe: {z}')
            self.get_logger().info(f'RESUME GRASP X={x:.4f} Y={y:.4f} Z={z:.4f}')
            above = ik(x, y, 0.115, q5)
            self.move(above, 3.0, 'LIFT')
            if self.gripper_pos is None or self.gripper_pos > 0.43: raise RuntimeError('box lost during lift')
            self.finish_sort(q5)
            return
        self.move(HOME, 4.0, 'HOME')
        self.samples.clear()
        self.get_logger().info('Waiting for stable defect detection...')
        sample_count = 4
        deadline = time.monotonic() + 20.0
        while len(self.samples) < sample_count and time.monotonic() < deadline: self.spin_for(0.1)
        if len(self.samples) < sample_count: raise RuntimeError('stable defect not detected')
        target = np.median(np.stack(self.samples[-sample_count:]), axis=0)
        spread = np.ptp(np.stack(self.samples[-sample_count:]), axis=0)
        if spread[0] > 0.012 or spread[1] > 0.012 or spread[2] > 0.20:
            raise RuntimeError(f'unstable target: spread={spread}')
        x, y, q5 = target
        if not (0.07 <= x <= 0.30 and abs(y) <= 0.23 and math.hypot(x, y) <= 0.31):
            raise RuntimeError(f'target outside workspace: X={x:.3f}, Y={y:.3f}')
        self.get_logger().info(f'TARGET X={x:.4f} Y={y:.4f} joint5={q5:.4f}')

        pitch = select_pick_pitch(x, y, q5)
        self.get_logger().info(f'PICK PITCH={math.degrees(pitch):.1f} deg')
        above = ik(x, y, 0.115, q5, pitch)
        self.move(above, 4.0, 'ABOVE BOX')
        above_actual_z = self.z()
        if above_actual_z is None or not 0.075 <= above_actual_z <= 0.135:
            raise RuntimeError(f'unsafe above-box Z: {above_actual_z}')
        # Compensate position-dependent gravity sag before issuing the single descent.
        pick_z_calc = float(np.clip(0.025 + (0.115 - above_actual_z), 0.026, 0.045))
        pick = ik(x, y, pick_z_calc, q5, pitch)
        self.move(pick, 2.7, 'DESCEND')  # one continuous descent
        z = self.z()
        if z is None or not 0.020 <= z <= 0.032: raise RuntimeError(f'unsafe pick Z: {z}')
        self.grip(0.340, 8.0, expect_stall=True)
        self.move(above, 3.0, 'LIFT')     # one continuous lift
        if self.gripper_pos is None or self.gripper_pos > 0.43: raise RuntimeError('box lost during lift')

        self.finish_sort(q5)

    def finish_sort(self, q5):

        sort_high = ik(*SORT_HIGH_XY, 0.105, q5)  # validated loaded-arm transport pose
        sort_drop = ik(*SORT_HIGH_XY, 0.085, q5)  # verified actual Z about 0.068 m
        self.move(sort_high, 4.5, 'TRANSPORT')
        if self.gripper_pos is None or self.gripper_pos > 0.43: raise RuntimeError('box lost in transport')
        self.move(sort_drop, 2.5, 'LOWER TO BIN')
        z = self.z()
        if z is None or z < 0.050: raise RuntimeError(f'unsafe drop Z: {z}')
        self.grip(0.724, 5.0)
        self.move(sort_high, 2.5, 'CLEAR BIN')
        self.move(HOME, 5.0, 'RETURN HOME')
        self.get_logger().info('PICK/SORT COMPLETE')


def main():
    rclpy.init(); node = Runner()
    try:
        node.run()
    except Exception as exc:
        node.get_logger().error(f'SEQUENCE STOPPED: {exc}')
        node.destroy_node(); rclpy.shutdown(); return 1
    node.destroy_node(); rclpy.shutdown(); return 0


if __name__ == '__main__':
    sys.exit(main())
