#!/usr/bin/env python3
"""Open the gripper only after measured MoveJ pitch-pregrasp validation."""

import math

from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


class GripperOpen(Node):
    def __init__(self):
        super().__init__('gripper_open')
        defaults = {
            'target_topic': '/camera_box_target',
            'joint_states_topic': '/joint_states',
            'gripper_action': '/gripper_controller/gripper_cmd',
            'base_frame': 'link0',
            'dry_run': True,
            'arm_joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'],
            'gripper_joint_name': 'gripper_joint_1',
            'open_position': 1.0,
            'minimum_open_position': 0.80,
            'open_tolerance': 0.05,
            'open_watchdog_timeout': 2.0,
            'open_watchdog_stable_time': 0.50,
            'open_watchdog_position_epsilon': 0.003,
            'max_effort': 10.0,
            'target_max_age': 30.0,
            'max_xy_error': 0.005,
            'min_pregrasp_z': 0.13,
            'max_pregrasp_z': 0.18,
            'min_pregrasp_pitch': 1.221730476,
            'max_pregrasp_pitch': 1.570796327,
            'fk_position_bias': [0.00126, 0.0, 0.00055],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.positions = {}
        self.target = None
        self.active = False
        self.goal_handle = None
        self.watchdog_started = None
        self.watchdog_stable_since = None
        self.watchdog_last_position = None
        self.watchdog_accept_cancel = False
        self.watchdog_cancel_requested = False
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.create_subscription(PoseStamped, self.p('target_topic'), self.on_target, 10)
        self.create_subscription(JointState, self.p('joint_states_topic'), self.on_joints, 10)
        self.client = ActionClient(self, GripperCommand, self.p('gripper_action'))
        self.create_service(Trigger, '~/confirm', self.on_confirm)
        self.create_timer(0.05, self.update_open_watchdog)
        self.report('ready; validate pitch pregrasp, latch a fresh target, then call ~/confirm')

    def p(self, name):
        return self.get_parameter(name).value

    def report(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def on_target(self, message):
        if not self.active and message.header.frame_id == self.p('base_frame'):
            self.target = message
            self.report(
                f'target latched x={message.pose.position.x:.4f}, '
                f'y={message.pose.position.y:.4f}')

    def on_joints(self, message):
        self.positions.update(zip(message.name, message.position))

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
            matrix[:3, :3] = [[cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]]
        elif axis == 'y':
            matrix[:3, :3] = [[cosine, 0, sine], [0, 1, 0], [-sine, 0, cosine]]
        else:
            matrix[:3, :3] = [[1, 0, 0], [0, cosine, -sine], [0, sine, cosine]]
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

    def validate(self):
        if self.target is None:
            return None, 'no camera target received'
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(
            self.target.header.stamp)).nanoseconds / 1e9
        if age > float(self.p('target_max_age')):
            return None, f'target is stale ({age:.1f}s)'
        names = list(self.p('arm_joint_names'))
        gripper = self.p('gripper_joint_name')
        if not all(name in self.positions for name in names + [gripper]):
            return None, 'joint feedback unavailable'
        joints = np.asarray([self.positions[name] for name in names], dtype=float)
        pose = self.fk(joints)
        pitch = float(np.sum(joints[1:4]))
        target_xy = np.asarray([
            self.target.pose.position.x, self.target.pose.position.y])
        xy_error = float(np.linalg.norm(pose[:2] - target_xy))
        if xy_error > float(self.p('max_xy_error')):
            return None, f'pregrasp XY error is {xy_error*1000:.2f}mm'
        if not float(self.p('min_pregrasp_z')) <= pose[2] <= float(self.p('max_pregrasp_z')):
            return None, f'not at pregrasp Z: {pose[2]:.4f}m'
        if not float(self.p('min_pregrasp_pitch')) <= pitch <= float(self.p('max_pregrasp_pitch')):
            return None, f'not at pregrasp pitch: {math.degrees(pitch):.2f}deg'
        return (pose, pitch, xy_error, float(self.positions[gripper])), ''

    def on_confirm(self, _request, response):
        if self.active:
            response.success, response.message = False, 'gripper command already active'
            return response
        state, error = self.validate()
        if error:
            response.success, response.message = False, error
            return response
        pose, pitch, xy_error, current = state
        summary = (
            f'pregrasp Z={pose[2]:.4f}m, XY error={xy_error*1000:.2f}mm, '
            f'pitch={math.degrees(pitch):.2f}deg, gripper={current:.4f}rad')
        if self.p('dry_run'):
            response.success = False
            response.message = f'dry_run=true; {summary}'
            self.report(response.message)
            return response
        if not self.client.server_is_ready():
            response.success, response.message = False, 'gripper action unavailable'
            return response
        goal = GripperCommand.Goal()
        goal.command.position = float(self.p('open_position'))
        goal.command.max_effort = float(self.p('max_effort'))
        self.active = True
        self.watchdog_started = self.get_clock().now()
        self.watchdog_stable_since = None
        self.watchdog_last_position = current
        self.watchdog_accept_cancel = False
        self.watchdog_cancel_requested = False
        self.client.send_goal_async(goal).add_done_callback(self.on_goal)
        response.success, response.message = True, f'gripper open started; {summary}'
        return response

    def on_goal(self, future):
        try:
            self.goal_handle = future.result()
        except Exception as error:
            self.active = False
            self.report(f'FAILED: gripper goal error: {error}')
            return
        if not self.goal_handle.accepted:
            self.active = False
            self.report('FAILED: gripper goal rejected')
            return
        self.goal_handle.get_result_async().add_done_callback(self.on_result)

    def update_open_watchdog(self):
        if (not self.active or self.goal_handle is None or
                self.watchdog_started is None or self.watchdog_cancel_requested):
            return
        position = self.positions.get(self.p('gripper_joint_name'))
        if position is None:
            return
        now = self.get_clock().now()
        epsilon = float(self.p('open_watchdog_position_epsilon'))
        if (self.watchdog_last_position is None or
                abs(position - self.watchdog_last_position) > epsilon):
            self.watchdog_last_position = position
            self.watchdog_stable_since = now
            return
        if self.watchdog_stable_since is None:
            self.watchdog_stable_since = now
            return
        elapsed = (now - self.watchdog_started).nanoseconds / 1e9
        stable = (now - self.watchdog_stable_since).nanoseconds / 1e9
        open_enough = (
            position >= float(self.p('minimum_open_position')) and
            abs(position - float(self.p('open_position'))) <=
            float(self.p('open_tolerance')))
        if (elapsed < float(self.p('open_watchdog_timeout')) or
                stable < float(self.p('open_watchdog_stable_time')) or
                not open_enough):
            return
        self.watchdog_accept_cancel = True
        self.watchdog_cancel_requested = True
        self.report(
            f'open watchdog accepted stable opening at {position:.4f}rad; '
            'cancelling non-terminating open action')
        self.goal_handle.cancel_goal_async()

    def on_result(self, future):
        self.active = False
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as error:
            self.report(f'FAILED: gripper result error: {error}')
            return
        accepted_watchdog_open = (
            wrapped.status == GoalStatus.STATUS_CANCELED and
            self.watchdog_accept_cancel)
        measured = self.positions.get(self.p('gripper_joint_name'))
        position = float(measured if accepted_watchdog_open else result.position)
        self.goal_handle = None
        self.watchdog_started = None
        self.watchdog_stable_since = None
        self.watchdog_last_position = None
        self.watchdog_accept_cancel = False
        self.watchdog_cancel_requested = False
        if (wrapped.status != GoalStatus.STATUS_SUCCEEDED and
                not accepted_watchdog_open):
            self.report(f'FAILED: gripper action status={wrapped.status}')
        elif not accepted_watchdog_open and not result.reached_goal:
            self.report(
                f'FAILED: open goal not reached; position={position:.4f}rad, '
                f'effort={result.effort:.2f}, stalled={result.stalled}')
        elif position < float(self.p('minimum_open_position')):
            self.report(f'FAILED: insufficient opening; position={position:.4f}rad')
        elif abs(position - float(self.p('open_position'))) > float(self.p('open_tolerance')):
            self.report(f'FAILED: open position error; position={position:.4f}rad')
        else:
            if accepted_watchdog_open:
                self.report(f'accepted watchdog open at {position:.4f}rad')
            self.report(f'COMPLETED: gripper fully open at {position:.4f}rad')


def main(args=None):
    rclpy.init(args=args)
    node = GripperOpen()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
