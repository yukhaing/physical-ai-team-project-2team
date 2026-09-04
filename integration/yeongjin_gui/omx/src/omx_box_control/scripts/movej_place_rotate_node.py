#!/usr/bin/env python3
"""Rotate a loaded OMX-F toward a homography-selected place direction."""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from movej_xy_approach_node import MoveJXyApproach


class MoveJPlaceRotate(MoveJXyApproach):
    def __init__(self):
        super().__init__()
        extras = {
            'gripper_joint_name': 'gripper_joint_1',
            'min_grasp_position': 0.05,
            'max_grasp_position': 0.60,
            'max_gripper_change': 0.05,
            'base_xy': [-0.00999, 0.0],
            'use_fixed_end_joint1': True,
            'fixed_end_joint1': -0.8759030299,
            'remembered_end_xyz': [0.1711, -0.2197, 0.1236],
            'max_step_angle': math.radians(5.0),
            'minimum_step_angle': math.radians(0.25),
            'load_compensation': [0.0, -0.100, -0.039, 0.0075, 0.0],
            'max_command_path_drop': 0.001,
            'minimum_z': 0.110,
            'max_actual_z_drop': 0.005,
            'max_non_joint1_change': math.radians(2.0),
            'max_direction_error': math.radians(3.0),
        }
        for name, value in extras.items():
            self.declare_parameter(name, value)
        self.start_joints = self.start_pose = None
        self.start_gripper = self.requested_step = None

    @staticmethod
    def wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def state(self):
        names = list(self.p('joint_names'))
        gripper_name = self.p('gripper_joint_name')
        if not all(name in self.positions for name in names + [gripper_name]):
            return None, None
        return (np.asarray([self.positions[name] for name in names], dtype=float),
                float(self.positions[gripper_name]))

    def on_confirm(self, _request, response):
        if self.active:
            response.success, response.message = False, 'place rotation already active'
            return response
        start, gripper = self.state()
        if start is None:
            response.success, response.message = False, 'joint or gripper feedback unavailable'
            return response
        fixed_mode = bool(self.p('use_fixed_end_joint1'))
        if not fixed_mode:
            if self.target is None:
                response.success, response.message = False, 'select a fresh homography target first'
                return response
            if self.target.header.frame_id != self.p('base_frame'):
                response.success, response.message = False, 'target frame does not match base frame'
                return response
            stamp = self.target.header.stamp
            age = self.get_clock().now().nanoseconds / 1e9 - (stamp.sec + stamp.nanosec / 1e9)
            if age < -0.5 or age > float(self.p('target_max_age')):
                response.success, response.message = False, f'target is stale ({age:.1f}s)'
                return response
        if not float(self.p('min_grasp_position')) <= gripper <= float(self.p('max_grasp_position')):
            response.success, response.message = False, f'gripper is outside grasp range ({gripper:.4f}rad)'
            return response

        start_pose = self.fk(start)
        base = np.asarray(self.p('base_xy'), dtype=float)
        if fixed_mode:
            requested = self.wrap(float(self.p('fixed_end_joint1')) - start[0])
        else:
            target_xy = np.asarray([self.target.pose.position.x, self.target.pose.position.y])
            if np.linalg.norm(target_xy - base) < 0.03:
                response.success, response.message = False, 'selected target is too close to joint1 axis'
                return response
            current_angle = math.atan2(start_pose[1] - base[1], start_pose[0] - base[0])
            target_angle = math.atan2(target_xy[1] - base[1], target_xy[0] - base[0])
            requested = self.wrap(target_angle - current_angle)
        if abs(requested) < float(self.p('minimum_step_angle')):
            response.success, response.message = True, 'already aligned with selected direction'
            return response
        limit = float(self.p('max_step_angle'))
        step = max(-limit, min(limit, requested))
        compensation = np.asarray(self.p('load_compensation'), dtype=float)
        if compensation.shape != start.shape:
            response.success, response.message = False, 'invalid load_compensation size'
            return response
        goal = start + compensation
        goal[0] = start[0] + step
        lower = np.asarray(self.p('joint_lower'), dtype=float) + float(self.p('joint_limit_margin'))
        upper = np.asarray(self.p('joint_upper'), dtype=float) - float(self.p('joint_limit_margin'))
        if np.any(goal < lower) or np.any(goal > upper):
            response.success, response.message = False, 'place rotation violates joint margins'
            return response
        path = self.path_positions(start, goal)
        path_min = min(point[2] for point in path)
        if path_min < float(self.p('minimum_z')):
            response.success, response.message = False, f'command path Z too low ({path_min:.4f}m)'
            return response
        if path_min < start_pose[2] - float(self.p('max_command_path_drop')):
            response.success, response.message = False, 'commanded path contains excessive Z drop'
            return response
        projected = start.copy()
        projected[0] += step
        projected_pose = self.fk(projected)
        mode = 'fixed endpoint' if fixed_mode else 'homography direction'
        details = (f'mode={mode}, requested={math.degrees(requested):.2f}deg, '
                   f'step={math.degrees(step):.2f}deg, '
                   f'start XYZ={[round(v, 4) for v in start_pose]}, projected XY='
                   f'{[round(v, 4) for v in projected_pose[:2]]}, path min={path_min:.4f}m, '
                   f'compensation={[round(v, 4) for v in compensation]}')
        if self.p('dry_run'):
            response.success, response.message = False, f'dry_run=true; {details}'
            self.report(response.message)
            return response
        message = JointTrajectory()
        message.joint_names = list(self.p('joint_names'))
        point = JointTrajectoryPoint()
        point.positions = goal.tolist()
        duration = float(self.p('move_duration'))
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)
        message.points = [point]
        self.move_pub.publish(message)
        self.goal = dict(zip(message.joint_names, goal))
        self.start_joints, self.start_pose = start, start_pose
        self.start_gripper, self.requested_step = gripper, step
        self.started = self.get_clock().now()
        self.stable_since = None
        self.active = True
        self.report(f'place rotation started; {details}')
        response.success, response.message = True, 'bounded place rotation started'
        return response

    def update(self):
        if not self.active:
            return
        now = self.get_clock().now()
        elapsed = (now - self.started).nanoseconds / 1e9
        if elapsed > float(self.p('timeout')):
            self.active = False
            self.report('FAILED: place rotation timeout; no next command published')
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
        actual, gripper = self.state()
        if actual is None:
            self.report('FAILED SAFE: feedback unavailable; no next command published')
            return
        pose = self.fk(actual)
        z_drop = self.start_pose[2] - pose[2]
        non_joint1 = np.max(np.abs(actual[1:] - self.start_joints[1:]))
        base = np.asarray(self.p('base_xy'), dtype=float)
        actual_angle = math.atan2(pose[1] - base[1], pose[0] - base[0])
        if self.p('use_fixed_end_joint1'):
            remaining = abs(self.wrap(float(self.p('fixed_end_joint1')) - actual[0]))
        else:
            target_xy = np.asarray([self.target.pose.position.x, self.target.pose.position.y])
            target_angle = math.atan2(target_xy[1] - base[1], target_xy[0] - base[0])
            remaining = abs(self.wrap(target_angle - actual_angle))
        moved = self.wrap(actual[0] - self.start_joints[0])
        summary = (f'XYZ={[round(v, 4) for v in pose]}, joint1 moved={math.degrees(moved):.2f}deg, '
                   f'Z drop={z_drop*1000:.1f}mm, max joint2-5 change='
                   f'{math.degrees(non_joint1):.2f}deg, remaining direction={math.degrees(remaining):.2f}deg, '
                   f'gripper={gripper:.4f}rad')
        failure = None
        if not float(self.p('min_grasp_position')) <= gripper <= float(self.p('max_grasp_position')):
            failure = 'gripper is outside grasp range'
        elif abs(gripper - self.start_gripper) > float(self.p('max_gripper_change')):
            failure = 'gripper changed excessively'
        elif pose[2] < float(self.p('minimum_z')):
            failure = 'actual Z is below minimum'
        elif z_drop > float(self.p('max_actual_z_drop')):
            failure = 'actual Z drop exceeds limit'
        elif non_joint1 > float(self.p('max_non_joint1_change')):
            failure = 'joint2-5 posture change exceeds limit'
        elif moved * self.requested_step <= 0.0:
            failure = 'joint1 moved in the wrong direction'
        if failure:
            self.report(f'FAILED SAFE: {failure}; {summary}; no next command published')
        else:
            self.report(f'COMPLETED STEP: {summary}; select/confirm again for another bounded step')


def main(args=None):
    rclpy.init(args=args)
    node = MoveJPlaceRotate()
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
