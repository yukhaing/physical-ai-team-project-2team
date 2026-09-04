#!/usr/bin/env python3
"""Execute one dynamically planned MoveJ PTP descent to a camera X/Y target."""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from movej_xy_approach_node import MoveJXyApproach


class MoveJSinglePtpDescent(MoveJXyApproach):
    """Plan and execute exactly one joint-space descent command."""

    def __init__(self):
        super().__init__()
        extras = {
            'target_z_min': 0.025,
            'target_z_max': 0.040,
            'max_path_xy_deviation': 0.030,
            'minimum_descent': 0.030,
            'maximum_descent': 0.090,
            'gripper_joint_name': 'gripper_joint_1',
            'gripper_gate_mode': 'open',
            'minimum_open_position': 0.80,
            'min_grasp_position': 0.005,
            'max_grasp_position': 0.70,
        }
        for name, value in extras.items():
            self.declare_parameter(name, value)
        self.start_z = None
        self.report(
            'SINGLE_PTP ready; one MoveJ command only; '
            'select a fresh target, then call ~/confirm')

    def current_joints(self):
        names = list(self.p('joint_names'))
        if not all(name in self.positions for name in names):
            return None
        return np.asarray([self.positions[name] for name in names], dtype=float)

    def on_confirm(self, _request, response):
        if self.active:
            response.success, response.message = False, 'single PTP descent already active'
            return response
        start, error = self.validate_request()
        if error:
            response.success, response.message = False, error
            return response

        gripper_name = str(self.p('gripper_joint_name'))
        if gripper_name not in self.positions:
            response.success, response.message = False, 'gripper feedback unavailable'
            return response
        gripper_position = float(self.positions[gripper_name])
        gate_mode = str(self.p('gripper_gate_mode'))
        if gate_mode == 'open' and gripper_position < float(self.p('minimum_open_position')):
            response.success, response.message = False, (
                f'gripper is not fully open: {gripper_position:.4f}rad')
            return response
        if gate_mode == 'grasp' and not (
                float(self.p('min_grasp_position')) <= gripper_position <=
                float(self.p('max_grasp_position'))):
            response.success, response.message = False, (
                f'gripper is not holding a box: {gripper_position:.4f}rad')
            return response
        if gate_mode not in ('open', 'grasp'):
            response.success, response.message = False, (
                f'unsupported gripper_gate_mode: {gate_mode}')
            return response

        target_xy = np.asarray([
            self.target.pose.position.x, self.target.pose.position.y], dtype=float)
        start_point = self.fk(start)
        try:
            goal = self.plan(start, target_xy)
        except Exception as exception:
            response.success, response.message = False, f'single PTP planning failed: {exception}'
            return response

        final = self.fk(goal)
        path = self.path_positions(start, goal)
        path_min_z = min(point[2] for point in path)
        path_max_xy_deviation = max(
            float(np.linalg.norm(point[:2] - target_xy)) for point in path)
        final_xy_error = float(np.linalg.norm(final[:2] - target_xy))
        planned_descent = float(start_point[2] - final[2])
        pitch = float(np.sum(goal[1:4]))

        if final_xy_error > float(self.p('max_xy_error')):
            response.success, response.message = False, (
                f'planned final XY error too large: {final_xy_error*1000:.2f}mm')
            return response
        if path_max_xy_deviation > float(self.p('max_path_xy_deviation')):
            response.success, response.message = False, (
                f'planned PTP path XY deviation too large: '
                f'{path_max_xy_deviation*1000:.2f}mm')
            return response
        if path_min_z + 1e-9 < float(self.p('min_path_z')):
            response.success, response.message = False, (
                f'planned PTP path Z too low: {path_min_z:.4f}m')
            return response
        if not (float(self.p('minimum_descent')) <= planned_descent <=
                float(self.p('maximum_descent'))):
            response.success, response.message = False, (
                f'planned descent is outside range: {planned_descent*1000:.1f}mm')
            return response

        duration = max(0.02, float(self.p('move_duration')))
        summary = (
            f'SINGLE_PTP planned q={[round(value, 5) for value in goal]}; '
            f'start Z={start_point[2]:.4f}m, final Z={final[2]:.4f}m, '
            f'descent={planned_descent*1000:.1f}mm, '
            f'final XY error={final_xy_error*1000:.2f}mm, '
            f'path max XY deviation={path_max_xy_deviation*1000:.2f}mm, '
            f'path min Z={path_min_z:.4f}m, pitch={math.degrees(pitch):.2f}deg, '
            f'duration={duration:.1f}s')
        if self.p('dry_run'):
            self.report(f'SINGLE_PTP DRY_RUN: {summary}')
            response.success, response.message = False, f'dry_run=true; {summary}'
            return response

        message = JointTrajectory()
        message.joint_names = list(self.p('joint_names'))
        point = JointTrajectoryPoint()
        point.positions = goal.tolist()
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1.0e9)
        message.points = [point]
        self.move_pub.publish(message)
        self.goal = dict(zip(message.joint_names, goal))
        self.start_z = float(start_point[2])
        self.started = self.get_clock().now()
        self.stable_since = None
        self.active = True
        self.report(f'SINGLE_PTP moving: {summary}')
        response.success, response.message = True, 'single MoveJ PTP descent started'
        return response

    def update(self):
        if not self.active:
            return
        now = self.get_clock().now()
        elapsed = (now - self.started).nanoseconds / 1e9
        actual = self.current_joints()
        if actual is None:
            self.active = False
            self.report('SINGLE_PTP FAILED SAFE: joint feedback disappeared; no next command')
            return

        errors = [abs(self.positions.get(name, math.inf) - value)
                  for name, value in self.goal.items()]
        maximum = max(errors)
        pose = self.fk(actual)
        target_xy = np.asarray([
            self.target.pose.position.x, self.target.pose.position.y], dtype=float)
        xy_error = float(np.linalg.norm(pose[:2] - target_xy))
        pitch = float(np.sum(actual[1:4]))
        descent = float(self.start_z - pose[2])

        if pose[2] < float(self.p('target_z_min')):
            self.active = False
            self.report(
                f'SINGLE_PTP FAILED SAFE: actual Z={pose[2]:.4f}m below '
                f'limit={float(self.p("target_z_min")):.4f}m; no next command')
            return
        if elapsed > float(self.p('timeout')):
            self.active = False
            self.report(
                f'SINGLE_PTP FAILED: timeout; joint error={math.degrees(maximum):.2f}deg, '
                f'XY error={xy_error*1000:.2f}mm, Z={pose[2]:.4f}m; no next command')
            return

        complete = (
            maximum <= float(self.p('joint_tolerance')) and
            xy_error <= float(self.p('max_xy_error')) and
            float(self.p('target_z_min')) <= pose[2] <= float(self.p('target_z_max')) and
            float(self.p('min_pitch')) <= pitch <= float(self.p('max_pitch')) and
            float(self.p('minimum_descent')) <= descent <=
            float(self.p('maximum_descent')))
        if not complete or elapsed < float(self.p('minimum_completion_time')):
            self.stable_since = None
            return
        if self.stable_since is None:
            self.stable_since = now
            return
        if (now - self.stable_since).nanoseconds / 1e9 < float(self.p('settle_time')):
            return

        self.active = False
        self.report(
            f'SINGLE_PTP COMPLETED: Z={pose[2]:.4f}m, '
            f'descent={descent*1000:.1f}mm, XY error={xy_error*1000:.2f}mm, '
            f'pitch={math.degrees(pitch):.2f}deg, '
            f'maximum joint error={math.degrees(maximum):.2f}deg')


def main(args=None):
    rclpy.init(args=args)
    node = MoveJSinglePtpDescent()
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
