#!/usr/bin/env python3
"""Feedback-gated loaded lift with bounded joint2/joint3 compensation."""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from movej_xy_approach_node import MoveJXyApproach


class MoveJLoadedLift(MoveJXyApproach):
    def __init__(self):
        super().__init__()
        extras = {
            'gripper_joint_name': 'gripper_joint_1',
            'min_grasp_position': 0.15,
            'max_grasp_position': 0.60,
            'max_gripper_change': 0.05,
            'min_start_z': 0.075,
            'max_start_z': 0.120,
            'min_completed_z': 0.130,
            'max_completed_z': 0.145,
            'min_actual_rise': 0.015,
            'max_actual_rise': 0.060,
            'max_command_xy_shift': 0.010,
            'max_actual_xy_shift': 0.030,
            'max_path_drop': 0.001,
            'max_command_z': 0.170,
            'joint_goal_compensation': [0.0, -0.06, -0.04, 0.0, 0.0],
            'reset_joint5_during_lift': True,
            'lift_target_joint5': 0.0,
            'maximum_lift_joint5_delta': 0.80,
            'lift_joint5_tolerance': 0.05,
        }
        for name, value in extras.items():
            self.declare_parameter(name, value)
        self.start_pose = None
        self.start_gripper = None
        self.nominal_goal = None

    def state(self):
        names = list(self.p('joint_names'))
        gripper_name = self.p('gripper_joint_name')
        if not all(name in self.positions for name in names + [gripper_name]):
            return None, None
        joints = np.asarray([self.positions[name] for name in names], dtype=float)
        return joints, float(self.positions[gripper_name])

    def validate_result(self):
        actual, gripper = self.state()
        if actual is None:
            return None, 'joint or gripper feedback unavailable'
        pose = self.fk(actual)
        rise = float(pose[2] - self.start_pose[2])
        xy_shift = float(np.linalg.norm(pose[:2] - self.start_pose[:2]))
        gripper_change = abs(gripper - self.start_gripper)
        joint5_error = abs(actual[4] - float(self.p('lift_target_joint5')))
        summary = (
            f'Z={pose[2]:.4f}m, rise={rise*1000:.1f}mm, '
            f'XY shift={xy_shift*1000:.2f}mm, '
            f'pitch={math.degrees(float(np.sum(actual[1:4]))):.2f}deg, '
            f'joint5={math.degrees(float(actual[4])):.2f}deg, '
            f'gripper={gripper:.4f}rad')
        if not float(self.p('min_grasp_position')) <= gripper <= float(self.p('max_grasp_position')):
            return summary, 'gripper is outside the grasp range'
        if gripper_change > float(self.p('max_gripper_change')):
            return summary, f'gripper changed by {gripper_change:.4f}rad'
        if not float(self.p('min_completed_z')) <= pose[2] <= float(self.p('max_completed_z')):
            return summary, 'actual Z is outside the completed band'
        if not float(self.p('min_actual_rise')) <= rise <= float(self.p('max_actual_rise')):
            return summary, 'actual rise is outside the safe band'
        if xy_shift > float(self.p('max_actual_xy_shift')):
            return summary, 'actual XY shift exceeds the gross-motion safety limit'
        if (bool(self.p('reset_joint5_during_lift')) and
                joint5_error > float(self.p('lift_joint5_tolerance'))):
            return summary, (
                f'joint5 reset error is too large: '
                f'{math.degrees(joint5_error):.1f}deg')
        return summary, ''

    def on_confirm(self, _request, response):
        if self.active:
            response.success, response.message = False, 'loaded lift already active'
            return response
        start, gripper = self.state()
        if start is None:
            response.success, response.message = False, 'joint or gripper feedback unavailable'
            return response
        start_pose = self.fk(start)
        if not float(self.p('min_grasp_position')) <= gripper <= float(self.p('max_grasp_position')):
            response.success, response.message = False, (
                f'gripper is not holding a box: {gripper:.4f}rad')
            return response
        if not float(self.p('min_start_z')) <= start_pose[2] <= float(self.p('max_start_z')):
            response.success, response.message = False, (
                f'start Z={start_pose[2]:.4f}m is outside loaded-lift range')
            return response
        target_xy = start_pose[:2].copy()
        target_joint5 = None
        if bool(self.p('reset_joint5_during_lift')):
            target_joint5 = float(self.p('lift_target_joint5'))
            joint5_delta = abs(target_joint5 - start[4])
            if joint5_delta > float(self.p('maximum_lift_joint5_delta')):
                response.success, response.message = False, (
                    f'joint5 reset change is too large: '
                    f'{math.degrees(joint5_delta):.1f}deg')
                return response
        try:
            nominal = self.plan(start, target_xy, target_joint5)
        except Exception as exception:
            response.success, response.message = False, f'nominal lift planning failed: {exception}'
            return response
        compensation = np.asarray(self.p('joint_goal_compensation'), dtype=float)
        if compensation.shape != nominal.shape:
            response.success, response.message = False, 'invalid joint_goal_compensation size'
            return response
        goal = nominal + compensation
        lower = np.asarray(self.p('joint_lower'), dtype=float) + float(self.p('joint_limit_margin'))
        upper = np.asarray(self.p('joint_upper'), dtype=float) - float(self.p('joint_limit_margin'))
        if np.any(goal < lower) or np.any(goal > upper):
            response.success, response.message = False, 'compensated goal violates joint margins'
            return response
        path = self.path_positions(start, goal)
        path_min = min(point[2] for point in path)
        command_pose = self.fk(goal)
        command_xy_shift = float(np.linalg.norm(command_pose[:2] - start_pose[:2]))
        if path_min < start_pose[2] - float(self.p('max_path_drop')):
            response.success, response.message = False, 'compensated path descends below the start'
            return response
        if command_pose[2] > float(self.p('max_command_z')):
            response.success, response.message = False, 'compensated command Z exceeds its limit'
            return response
        if command_xy_shift > float(self.p('max_command_xy_shift')):
            response.success, response.message = False, 'compensated command XY shift exceeds limit'
            return response
        details = (
            f'start Z={start_pose[2]:.4f}m, nominal Z={self.fk(nominal)[2]:.4f}m, '
            f'command Z={command_pose[2]:.4f}m, path min={path_min:.4f}m, '
            f'command XY shift={command_xy_shift*1000:.2f}mm, '
            f'joint5={math.degrees(float(goal[4])):.2f}deg, '
            f'compensation={[round(value, 4) for value in compensation]}')
        if self.p('dry_run'):
            response.success, response.message = False, f'dry_run=true; {details}'
            self.report(response.message)
            return response
        message = JointTrajectory()
        message.joint_names = list(self.p('joint_names'))
        point = JointTrajectoryPoint()
        point.positions = goal.tolist()
        duration = max(0.02, float(self.p('move_duration')))
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)
        message.points = [point]
        self.move_pub.publish(message)
        self.goal = dict(zip(message.joint_names, goal))
        self.start_pose = start_pose
        self.start_gripper = gripper
        self.nominal_goal = nominal
        self.started = self.get_clock().now()
        self.stable_since = None
        self.active = True
        self.report(f'loaded lift started; {details}')
        response.success, response.message = True, 'compensated loaded lift started'
        return response

    def update(self):
        if not self.active:
            return
        now = self.get_clock().now()
        elapsed = (now - self.started).nanoseconds / 1e9
        if elapsed > float(self.p('timeout')):
            self.active = False
            summary, failure = self.validate_result()
            self.report(f'FAILED: timeout; {failure or "completion not stable"}; {summary}')
            return
        if elapsed < float(self.p('minimum_completion_time')):
            self.stable_since = None
            return
        if self.stable_since is None:
            self.stable_since = now
            return
        if (now - self.stable_since).nanoseconds / 1e9 < float(self.p('settle_time')):
            return
        self.active = False
        summary, failure = self.validate_result()
        if failure:
            self.report(f'FAILED SAFE: {failure}; {summary}; no next command published')
        else:
            self.report(f'COMPLETED: {summary}; holding lifted pose')


def main(args=None):
    rclpy.init(args=args)
    node = MoveJLoadedLift()
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
