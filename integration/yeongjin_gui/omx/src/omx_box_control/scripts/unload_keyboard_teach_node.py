#!/usr/bin/env python3
"""Interactive XY teaching for the standalone unloading OMX."""

import math
from pathlib import Path
import select
import sys
import termios
import tty

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.optimize import minimize
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


HELP = """
Unload source teaching
  G   : lock tray, then move parking -> high staging -> teach start
  B   : return arm to camera-clear vision parking
  W/S : X +/-       A/D : Y +/-       R/F : Z +/-
  1   : 1 mm step   5   : 5 mm step   0 : 10 mm step
  P   : print actual TCP and stable Beagle tray reference
  V   : save robot XY/joint5 + tray reference together
  H   : show help    X or ESC : exit

Put Beagle with its black tray at the normal dock position.
Align the empty gripper center above the box. Z is not saved from the camera.
Wait for each move to settle before sending the next key.
"""


class UnloadKeyboardTeach(Node):
    def __init__(self):
        super().__init__('unload_keyboard_teach')
        defaults = {
            'movej_topic': '/unload_omx/omx_movej_controller/movej',
            'joint_states_topic': '/unload_omx/joint_states',
            'raw_target_topic': '/unload_omx/vision_raw_target',
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'],
            'joint_lower': [-4.71239, -2.0944, -2.0944, -1.74533, -4.71239],
            'joint_upper': [6.28319, 1.5708, 1.5708, 1.74533, 4.71239],
            'joint_limit_margin': 0.06,
            'minimum_pitch': 1.047197551,
            'maximum_pitch': 1.483529864,
            'preferred_pitch': 1.221730476,
            'fk_position_bias': [0.00126, 0.0, 0.00055],
            'minimum_reach_radius': 0.10,
            'maximum_reach_radius': 0.29,
            'minimum_teach_z': 0.15599,
            'source_approach_z': 0.19253,
            'bootstrap_source_xy': [0.180, 0.000],
            'staging_positions': [0.008422, -0.387875, -0.181383, 1.639417, 0.0],
            'parking_positions': [0.067495, -2.009515, 1.713457, 1.647495, -0.067495],
            'staging_duration': 13.0,
            'parking_duration': 8.0,
            'approach_duration': 8.0,
            'maximum_start_joint_delta': 2.0,
            'maximum_approach_joint_step': 1.0,
            'minimum_approach_path_z': 0.130,
            'maximum_teach_z': 0.230,
            'maximum_ik_error': 0.006,
            'camera_timeout': 1.0,
            'feedback_timeout': 0.5,
            'move_duration': 1.5,
            'joint_tolerance': 0.060,
            'staging_joint_tolerance': 0.060,
            'maximum_joint_step': 0.35,
            'step': 0.005,
            'profile_path': '/root/omx_box_project_ws/integration/yeongjin_gui/runtime/calibration/unload_source_teach.yaml',
            'calibration_file': '/root/omx_box_project_ws/integration/yeongjin_gui/runtime/calibration/unload_active.yaml',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.positions = {}
        self.feedback_time = None
        self.raw_camera_xy = None
        self.raw_camera_time = None
        self.raw_marker_angle = None
        self.locked_reference_xy = None
        self.locked_reference_angle = None
        self.step = float(self.p('step'))
        self.motion_goal = None
        self.motion_started = None
        self.motion_duration = 0.0
        self.pending_camera_start = False
        self.pending_unlock_after_parking = False
        self.teach_target_xyz = None
        self.move_pub = self.create_publisher(
            JointTrajectory, str(self.p('movej_topic')), 10)
        self.create_subscription(
            JointState, str(self.p('joint_states_topic')), self.on_joints, 10)
        self.create_subscription(
            PoseStamped, str(self.p('raw_target_topic')), self.on_raw_target, 10)

    def p(self, name):
        return self.get_parameter(name).value

    def on_joints(self, message):
        self.positions.update(zip(message.name, message.position))
        self.feedback_time = self.get_clock().now()
        if self.motion_goal is not None:
            actual = self.current_joints()
            elapsed = ((self.get_clock().now() - self.motion_started).nanoseconds / 1.0e9
                       if self.motion_started is not None else 0.0)
            parking_clear = (
                self.pending_unlock_after_parking and elapsed >= self.motion_duration and
                self.fresh_camera_reference() is not None)
            maximum_error = (float(np.max(np.abs(actual - self.motion_goal)))
                             if actual is not None else math.inf)
            staging_reached = (
                self.pending_camera_start and elapsed >= self.motion_duration and
                maximum_error <= float(self.p('staging_joint_tolerance')))
            reached_goal = (
                actual is not None and elapsed >= self.motion_duration and
                maximum_error <= float(self.p('joint_tolerance')))
            if parking_clear or staging_reached or reached_goal:
                self.motion_goal = None
                self.motion_started = None
                xyz = self.fk(actual)
                result = 'VISION_CLEAR' if parking_clear else 'SETTLED'
                print(f'{result} TCP x={xyz[0]:.5f}, y={xyz[1]:.5f}, z={xyz[2]:.5f} m')
                if self.pending_camera_start:
                    self.pending_camera_start = False
                    self.command_camera_start()
                elif self.pending_unlock_after_parking:
                    self.pending_unlock_after_parking = False
                    self.locked_reference_xy = None
                    self.locked_reference_angle = None
                    print('VISION PARKING ready; tray reference unlocked for fresh detection.')

    def on_raw_target(self, message):
        xy = np.asarray([message.pose.position.x, message.pose.position.y], dtype=float)
        quaternion = message.pose.orientation
        angle = 2.0 * math.atan2(float(quaternion.z), float(quaternion.w))
        if np.all(np.isfinite(xy)) and math.isfinite(angle):
            self.raw_camera_xy = xy
            self.raw_marker_angle = angle
            self.raw_camera_time = self.get_clock().now()

    @staticmethod
    def translation(x, y, z):
        matrix = np.eye(4)
        matrix[:3, 3] = [x, y, z]
        return matrix

    @staticmethod
    def rotation(axis, angle):
        cosine, sine = math.cos(angle), math.sin(angle)
        matrix = np.eye(4)
        if axis == 'z':
            matrix[:3, :3] = [
                [cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]]
        elif axis == 'y':
            matrix[:3, :3] = [
                [cosine, 0, sine], [0, 1, 0], [-sine, 0, cosine]]
        else:
            matrix[:3, :3] = [
                [1, 0, 0], [0, cosine, -sine], [0, sine, cosine]]
        return matrix

    def fk(self, joints):
        q1, q2, q3, q4, q5 = joints
        matrix = (
            self.translation(-0.01125, 0.0, 0.034) @ self.rotation('z', q1) @
            self.translation(0.0, 0.0, 0.0635) @ self.rotation('y', q2) @
            self.translation(0.0415, 0.0, 0.11315) @ self.rotation('y', q3) @
            self.translation(0.162, 0.0, 0.0) @ self.rotation('y', q4) @
            self.translation(0.0287, 0.0, 0.0) @ self.rotation('x', q5) @
            self.translation(0.09193, -0.0016, 0.0))
        return matrix[:3, 3] + np.asarray(self.p('fk_position_bias'), dtype=float)

    def current_joints(self):
        names = list(self.p('joint_names'))
        if not all(name in self.positions for name in names):
            return None
        return np.asarray([self.positions[name] for name in names], dtype=float)

    def feedback_ok(self):
        if self.feedback_time is None or self.current_joints() is None:
            print('BLOCKED: unloading OMX joint feedback is unavailable.')
            return False
        age = (self.get_clock().now() - self.feedback_time).nanoseconds / 1.0e9
        if age > float(self.p('feedback_timeout')):
            print(f'BLOCKED: joint feedback is stale ({age:.2f}s).')
            return False
        return True

    def solve_pose(self, seed, target):
        lower = np.asarray(self.p('joint_lower'), dtype=float)
        upper = np.asarray(self.p('joint_upper'), dtype=float)
        margin = float(self.p('joint_limit_margin'))
        q5 = float(seed[4])
        target_pitch = float(np.clip(np.sum(seed[1:4]),
                                    float(self.p('minimum_pitch')),
                                    float(self.p('maximum_pitch'))))
        target = np.asarray(target, dtype=float)
        bias = np.asarray(self.p('fk_position_bias'), dtype=float)
        radial_x = target[0] + 0.01125 - bias[0]
        radial_y = target[1] - bias[1]
        lateral = -0.0016 * math.cos(q5)
        radius = math.hypot(radial_x, radial_y)
        if radius <= abs(lateral):
            raise RuntimeError('target is too close to joint1 axis')
        local_x = math.sqrt(radius ** 2 - lateral ** 2)
        q1 = math.atan2(radial_y, radial_x) - math.atan2(lateral, local_x)
        if not lower[0] + margin <= q1 <= upper[0] - margin:
            raise RuntimeError('joint1 safety margin violation')
        bounds = list(zip(lower[1:4] + margin, upper[1:4] - margin))

        def full(values):
            return np.r_[q1, values, q5]

        def objective(values):
            goal = full(values)
            error = self.fk(goal) - target
            pitch_error = float(np.sum(goal[1:4])) - target_pitch
            delta = goal[1:4] - seed[1:4]
            return (30000.0 * float(error @ error) + 2.0 * pitch_error ** 2 +
                    0.05 * float(delta @ delta))

        def constraints(values):
            pitch = float(np.sum(full(values)[1:4]))
            return np.asarray([
                pitch - float(self.p('minimum_pitch')),
                float(self.p('maximum_pitch')) - pitch])

        candidates = []
        for values in (seed[1:4], np.asarray([-0.3, 0.3, 1.0]),
                       np.asarray([-0.6, 0.6, 1.0])):
            result = minimize(
                objective, values, method='SLSQP', bounds=bounds,
                constraints=[{'type': 'ineq', 'fun': constraints}],
                options={'maxiter': 400, 'ftol': 1e-11})
            if result.success and np.all(np.isfinite(result.x)):
                goal = full(result.x)
                candidates.append((float(np.linalg.norm(self.fk(goal) - target)), goal))
        if not candidates:
            raise RuntimeError('constrained IK found no feasible solution')
        error, goal = min(candidates, key=lambda item: item[0])
        if error > float(self.p('maximum_ik_error')):
            raise RuntimeError(f'IK error is too large: {error*1000.0:.2f}mm')
        return goal

    def publish_goal(self, goal, duration, description):
        message = JointTrajectory(joint_names=list(self.p('joint_names')))
        point = JointTrajectoryPoint(positions=[float(value) for value in goal])
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1.0e9)
        message.points = [point]
        self.motion_goal = np.asarray(goal, dtype=float)
        self.motion_started = self.get_clock().now()
        self.motion_duration = float(duration)
        self.move_pub.publish(message)
        print(f'MOVE {description}; duration={duration:.1f}s')

    def go_to_teach_start(self):
        if self.motion_goal is not None:
            print('WAIT: previous move has not settled.')
            return
        if not self.feedback_ok():
            return
        camera = self.camera_reference()
        if camera is None or self.raw_marker_angle is None:
            print('BLOCKED: stable Beagle tray reference is unavailable.')
            return
        self.locked_reference_xy = camera.copy()
        self.locked_reference_angle = float(self.raw_marker_angle)
        print(f'LOCKED tray reference x={camera[0]:.5f}, y={camera[1]:.5f}')
        current = self.current_joints()
        staging = np.asarray(self.p('staging_positions'), dtype=float)
        delta = float(np.max(np.abs(current - staging)))
        if delta > float(self.p('maximum_start_joint_delta')):
            print(f'BLOCKED too far from staging: {math.degrees(delta):.1f}deg')
            return
        self.pending_camera_start = True
        self.teach_target_xyz = None
        self.publish_goal(
            staging, float(self.p('staging_duration')),
            'to validated unloading staging')

    def return_to_vision_parking(self):
        if self.motion_goal is not None:
            print('WAIT: previous move has not settled.')
            return
        if not self.feedback_ok():
            return
        current = self.current_joints()
        parking = np.asarray(self.p('parking_positions'), dtype=float)
        delta = float(np.max(np.abs(current - parking)))
        if delta > float(self.p('maximum_start_joint_delta')):
            print(f'BLOCKED too far from vision parking: {math.degrees(delta):.1f}deg')
            return
        self.pending_camera_start = False
        self.pending_unlock_after_parking = True
        self.teach_target_xyz = None
        self.publish_goal(
            parking, float(self.p('parking_duration')),
            'to camera-clear vision parking')

    def command_camera_start(self):
        camera = self.camera_reference()
        if camera is None or not self.feedback_ok():
            print('STOPPED at staging: stable Beagle tray reference became unavailable.')
            return
        seed = self.current_joints()
        bootstrap = np.asarray(self.p('bootstrap_source_xy'), dtype=float)
        target = np.asarray([
            bootstrap[0], bootstrap[1], float(self.p('source_approach_z'))])
        center = np.asarray([-0.00999, 0.0])
        radius = float(np.linalg.norm(target[:2] - center))
        if not float(self.p('minimum_reach_radius')) <= radius <= float(
                self.p('maximum_reach_radius')):
            print(f'STOPPED at staging: camera target radius={radius:.4f}m is unsafe.')
            return
        try:
            goal = self.solve_pose(seed, target)
        except RuntimeError as error:
            print(f'STOPPED at staging: {error}')
            return
        joint_step = float(np.max(np.abs(goal - seed)))
        if joint_step > float(self.p('maximum_approach_joint_step')):
            print(f'STOPPED at staging: approach joint step '
                  f'{math.degrees(joint_step):.1f}deg is excessive.')
            return
        minimum_z = min(float(self.fk(seed + alpha * (goal - seed))[2])
                        for alpha in np.linspace(0.0, 1.0, 101))
        if minimum_z < float(self.p('minimum_approach_path_z')):
            print(f'STOPPED at staging: approach path Z={minimum_z:.4f}m is unsafe.')
            return
        self.teach_target_xyz = target.copy()
        self.publish_goal(
            goal, float(self.p('approach_duration')),
            f'to safe box-area teach start x={target[0]:.5f}, '
            f'y={target[1]:.5f}, z={target[2]:.5f}')

    def jog(self, dx, dy, dz):
        if self.motion_goal is not None:
            print('WAIT: previous move has not settled.')
            return
        if not self.feedback_ok():
            return
        seed = self.current_joints()
        # Keep Cartesian teaching anchored to the last commanded pose. Using
        # measured FK here accumulates MoveJ tracking error, making Z sag even
        # during nominally X/Y-only jogs.
        target = (self.teach_target_xyz.copy() if self.teach_target_xyz is not None
                  else self.fk(seed))
        target += [dx, dy, dz]
        center = np.asarray([-0.00999, 0.0])
        radius = float(np.linalg.norm(target[:2] - center))
        if not float(self.p('minimum_reach_radius')) <= radius <= float(
                self.p('maximum_reach_radius')):
            print(f'BLOCKED outside reach radius: {radius:.4f}m')
            return
        if not float(self.p('minimum_teach_z')) <= target[2] <= float(
                self.p('maximum_teach_z')):
            print(f'BLOCKED teach Z={target[2]:.4f}m outside safe teaching band.')
            return
        try:
            goal = self.solve_pose(seed, target)
        except RuntimeError as error:
            print(f'BLOCKED: {error}')
            return
        joint_step = float(np.max(np.abs(goal - seed)))
        if joint_step > float(self.p('maximum_joint_step')):
            print(f'BLOCKED excessive joint step: {math.degrees(joint_step):.1f}deg')
            return
        self.teach_target_xyz = target.copy()
        self.publish_goal(
            goal, float(self.p('move_duration')),
            f'jog target x={target[0]:.5f}, y={target[1]:.5f}, '
            f'z={target[2]:.5f}, step={self.step*1000.0:.0f}mm')

    def fresh_camera_reference(self):
        if self.raw_camera_time is None or self.raw_camera_xy is None:
            return None
        age = (self.get_clock().now() - self.raw_camera_time).nanoseconds / 1.0e9
        return (self.raw_camera_xy.copy()
                if age <= float(self.p('camera_timeout')) else None)

    def camera_reference(self):
        fresh = self.fresh_camera_reference()
        if fresh is not None:
            return fresh
        return (None if self.locked_reference_xy is None else
                self.locked_reference_xy.copy())

    def reference_angle(self):
        if self.locked_reference_angle is not None:
            return float(self.locked_reference_angle)
        return self.raw_marker_angle

    def print_actual(self):
        if not self.feedback_ok():
            return None
        joints = self.current_joints()
        xyz = self.fk(joints)
        camera = self.camera_reference()
        angle = self.reference_angle()
        marker_text = ('unavailable' if camera is None or angle is None else
                       f'x={camera[0]:.5f}, y={camera[1]:.5f}, '
                       f'angle={math.degrees(angle):.2f}deg')
        print(f'ACTUAL TCP x={xyz[0]:.5f}, y={xyz[1]:.5f}, z={xyz[2]:.5f} m; '
              f'LANDMARK {marker_text}')
        return xyz

    def save(self):
        if self.motion_goal is not None:
            print('BLOCKED: wait until the last motion settles.')
            return
        xyz = self.print_actual()
        camera = self.camera_reference()
        reference_angle = self.reference_angle()
        if xyz is None or camera is None or reference_angle is None:
            print('BLOCKED: fresh stable Beagle tray reference is required.')
            return
        if not float(self.p('minimum_teach_z')) <= xyz[2] <= float(
                self.p('maximum_teach_z')):
            print(f'BLOCKED save: first press G; TCP Z={xyz[2]:.4f}m is outside teaching band.')
            return
        path = Path(str(self.p('profile_path')))
        path.parent.mkdir(parents=True, exist_ok=True)
        content = (
            '# Unloading source teaching: robot pick point relative to Beagle tray.\n'
            f'taught_source_xy: [{xyz[0]:.8f}, {xyz[1]:.8f}]\n'
            f'reference_landmark_xy: [{camera[0]:.8f}, {camera[1]:.8f}]\n'
            f'reference_marker_angle: {reference_angle:.10f}\n'
            f'taught_joint5: {self.current_joints()[4]:.10f}\n'
            f'taught_tcp_z: {xyz[2]:.8f}\n'
            f'calibration_file: {self.p("calibration_file")}\n')
        temporary = path.with_suffix(path.suffix + '.tmp')
        temporary.write_text(content, encoding='utf-8')
        temporary.replace(path)
        print(f'SAVED {path}')
        print(f'  robot XY = [{xyz[0]:.5f}, {xyz[1]:.5f}]')
        print(f'  tray reference XY = [{camera[0]:.5f}, {camera[1]:.5f}]')
        print(f'  tray reference angle = {math.degrees(reference_angle):.2f}deg')
        print('Exit teaching and restart the unloading process to load this profile.')


def main(args=None):
    if not sys.stdin.isatty():
        raise RuntimeError('unload keyboard teach requires an interactive terminal')
    rclpy.init(args=args)
    node = UnloadKeyboardTeach()
    old_settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            readable, _, _ = select.select([sys.stdin], [], [], 0.02)
            if not readable:
                continue
            key = sys.stdin.read(1).lower()
            if key in ('x', '\x1b'):
                break
            if key == 'h':
                print(HELP)
            elif key == '1':
                node.step = 0.001
                print('Step = 1mm')
            elif key == '5':
                node.step = 0.005
                print('Step = 5mm')
            elif key == '0':
                node.step = 0.010
                print('Step = 10mm')
            elif key == 'g':
                node.go_to_teach_start()
            elif key == 'b':
                node.return_to_vision_parking()
            elif key == 'p':
                node.print_actual()
            elif key == 'v':
                node.save()
            elif key in 'wsadrf':
                delta = {
                    'w': (node.step, 0.0, 0.0),
                    's': (-node.step, 0.0, 0.0),
                    'a': (0.0, node.step, 0.0),
                    'd': (0.0, -node.step, 0.0),
                    'r': (0.0, 0.0, node.step),
                    'f': (0.0, 0.0, -node.step),
                }[key]
                node.jog(*delta)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
