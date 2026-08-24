#!/usr/bin/env python3
"""Plan a position-priority MoveJ approach with a minimum Cartesian Z constraint."""

import math

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.optimize import minimize
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveJXyApproach(Node):
    def __init__(self):
        super().__init__('movej_xy_approach')
        defaults = {
            'target_topic': '/camera_box_target',
            'movej_topic': '/omx_movej_controller/movej',
            'joint_states_topic': '/joint_states',
            'controller_error_topic': '/omx_movej_controller/controller_error',
            'base_frame': 'link0',
            'dry_run': True,
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'],
            'staging_positions': [0.0, -0.467, 0.376, 1.291, 0.0],
            'staging_ignored_joint_names': [''],
            'joint_lower': [-4.71239, -2.0944, -2.0944, -1.74533, -4.71239],
            'joint_upper': [6.28319, 1.5708, 1.5708, 1.74533, 4.71239],
            'joint_limit_margin': 0.06,
            'staging_joint_tolerance': 0.04,
            'require_staging': True,
            'require_start_at_target': False,
            'start_xy_tolerance': 0.010,
            'start_min_z': 0.13,
            'require_start_pitch': False,
            'start_min_pitch': -3.14159,
            'start_max_pitch': 3.14159,
            'min_path_z': 0.12,
            'min_final_z': 0.12,
            'require_actual_target_z': False,
            'actual_target_z_tolerance': 0.01,
            'use_target_z': False,
            'target_z': 0.13,
            'z_tolerance': 0.001,
            'z_weight': 0.0,
            'min_pitch': -3.14159,
            'max_pitch': 3.14159,
            'preferred_pitch': 0.0,
            'pitch_weight': 0.0,
            'path_samples': 51,
            'max_xy_error': 0.005,
            'target_max_age': 30.0,
            'min_x': 0.08,
            'max_x': 0.32,
            'max_abs_y': 0.25,
            'fk_position_bias': [0.00126, 0.0, 0.00055],
            'xy_weight': 20000.0,
            'joint_delta_weight': 0.02,
            'candidate_xy_slack': 0.0,
            'height_reward': 0.01,
            'move_duration': 6.0,
            'minimum_completion_time': 6.0,
            'joint_tolerance': 0.03,
            'settle_time': 0.30,
            'timeout': 15.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.positions = {}
        self.target = None
        self.goal = None
        self.started = None
        self.stable_since = None
        self.active = False
        self.move_pub = self.create_publisher(JointTrajectory, self.p('movej_topic'), 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.create_subscription(PoseStamped, self.p('target_topic'), self.on_target, 10)
        self.create_subscription(JointState, self.p('joint_states_topic'), self.on_joints, 10)
        self.create_subscription(String, self.p('controller_error_topic'), self.on_error, 10)
        self.create_service(Trigger, '~/confirm', self.on_confirm)
        self.create_service(Trigger, '~/cancel', self.on_cancel)
        self.create_timer(0.05, self.update)
        self.report('ready; reach MoveJ staging, select a fresh target, then call ~/confirm')

    def p(self, name):
        return self.get_parameter(name).value

    def report(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def on_target(self, message):
        if self.active or message.header.frame_id != self.p('base_frame'):
            return
        self.target = message
        point = message.pose.position
        z_mode = (f'Z target={float(self.p("target_z")):.4f}m'
                  if self.p('use_target_z') else 'Z will be optimized')
        self.report(f'target latched x={point.x:.4f}, y={point.y:.4f}; {z_mode}')

    def on_joints(self, message):
        self.positions.update(zip(message.name, message.position))

    def on_error(self, message):
        if self.active:
            self.report(f'WARNING: {message.data}; validating actual joint convergence')

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

    def path_positions(self, start, goal):
        samples = max(2, int(self.p('path_samples')))
        return [self.fk(start + alpha * (goal - start)) for alpha in np.linspace(0.0, 1.0, samples)]

    def plan(self, start, target_xy):
        lower = np.asarray(self.p('joint_lower'), dtype=float)
        upper = np.asarray(self.p('joint_upper'), dtype=float)
        margin = float(self.p('joint_limit_margin'))
        bounds = list(zip(lower[1:4] + margin, upper[1:4] - margin))
        fixed_q5 = float(start[4])

        # Joint1 only rotates the planar arm about the base.  Solve it from the
        # requested radial direction instead of making SLSQP discover it.
        bias = np.asarray(self.p('fk_position_bias'), dtype=float)
        radial_x = target_xy[0] + 0.01125 - bias[0]
        radial_y = target_xy[1] - bias[1]
        radial_distance = math.hypot(radial_x, radial_y)
        lateral_tool_offset = -0.0016
        if radial_distance <= abs(lateral_tool_offset):
            raise RuntimeError('target is too close to the joint1 axis')
        local_x = math.sqrt(radial_distance ** 2 - lateral_tool_offset ** 2)
        fixed_q1 = (math.atan2(radial_y, radial_x) -
                    math.atan2(lateral_tool_offset, local_x))
        if not lower[0] + margin <= fixed_q1 <= upper[0] - margin:
            raise RuntimeError(f'joint1 solution {fixed_q1:.3f}rad violates its margin')

        def full(values):
            return np.r_[fixed_q1, values, fixed_q5]

        def objective(values):
            goal = full(values)
            point = self.fk(goal)
            xy_error = point[:2] - target_xy
            delta = goal[1:4] - start[1:4]
            return (float(self.p('xy_weight')) * float(xy_error @ xy_error) +
                    float(self.p('z_weight')) *
                    (point[2] - float(self.p('target_z'))) ** 2 +
                    float(self.p('joint_delta_weight')) * float(delta @ delta) -
                    float(self.p('height_reward')) * (point[2] - float(self.p('min_path_z'))) +
                    float(self.p('pitch_weight')) *
                    (float(np.sum(goal[1:4])) - float(self.p('preferred_pitch'))) ** 2)

        def clearance(values):
            goal = full(values)
            path_clearance = [
                point[2] - float(self.p('min_path_z'))
                for point in self.path_positions(start, goal)]
            pitch = float(np.sum(goal[1:4]))
            constraints = path_clearance + [
                self.fk(goal)[2] - float(self.p('min_final_z')),
                pitch - float(self.p('min_pitch')),
                float(self.p('max_pitch')) - pitch,
            ]
            if self.p('use_target_z'):
                final_z = self.fk(goal)[2]
                constraints.extend([
                    final_z - (float(self.p('target_z')) - float(self.p('z_tolerance'))),
                    (float(self.p('target_z')) + float(self.p('z_tolerance'))) - final_z,
                ])
            return np.asarray(constraints)

        seeds = [
            start[1:4],
            np.asarray([-0.30, 0.30, 1.00]),
            np.asarray([-0.60, 0.60, 1.00]),
        ]
        candidates = []
        for seed in seeds:
            result = minimize(
                objective, seed, method='SLSQP', bounds=bounds,
                constraints=[{'type': 'ineq', 'fun': clearance}],
                options={'maxiter': 300, 'ftol': 1e-10})
            if result.success and np.all(np.isfinite(result.x)):
                candidate = full(result.x)
                xy_error = float(np.linalg.norm(
                    self.fk(candidate)[:2] - target_xy))
                candidates.append((xy_error, float(result.fun), candidate))
        if candidates:
            # A feasible SLSQP result can still miss the Cartesian target
            # because XY accuracy is an objective, not a hard constraint.
            # Compare every seed instead of returning the first local result.
            best_xy = min(item[0] for item in candidates)
            xy_slack = max(0.0, float(self.p('candidate_xy_slack')))
            if xy_slack > 0.0:
                near_best = [
                    item for item in candidates
                    if item[0] <= best_xy + xy_slack]
                return min(near_best, key=lambda item: item[1])[2]
            return min(candidates, key=lambda item: (item[0], item[1]))[2]
        raise RuntimeError('constrained IK found no feasible solution')

    def validate_request(self):
        names = list(self.p('joint_names'))
        if self.target is None:
            return None, 'no camera target received'
        if not all(name in self.positions for name in names):
            return None, 'joint feedback unavailable'
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(self.target.header.stamp)).nanoseconds / 1e9
        if age > float(self.p('target_max_age')):
            return None, f'target is stale ({age:.1f}s)'
        point = self.target.pose.position
        if not (math.isfinite(point.x) and math.isfinite(point.y) and
                float(self.p('min_x')) <= point.x <= float(self.p('max_x')) and
                abs(point.y) <= float(self.p('max_abs_y'))):
            return None, 'target X/Y is outside workspace'
        start = np.asarray([self.positions[name] for name in names], dtype=float)
        if self.p('require_staging'):
            staging = np.asarray(self.p('staging_positions'), dtype=float)
            ignored = {name for name in self.p('staging_ignored_joint_names') if name}
            unknown = ignored.difference(names)
            if unknown:
                return None, f'staging_ignored_joint_names contains unknown joints: {sorted(unknown)}'
            checked = [index for index, name in enumerate(names) if name not in ignored]
            if not checked:
                return None, 'staging gate cannot ignore every joint'
            staging_error = float(np.max(np.abs(start[checked] - staging[checked])))
            if staging_error > float(self.p('staging_joint_tolerance')):
                return None, f'not at MoveJ staging; maximum joint error={math.degrees(staging_error):.2f}deg'
        if self.p('require_start_at_target'):
            start_point = self.fk(start)
            start_xy_error = float(np.linalg.norm(start_point[:2] - [point.x, point.y]))
            if start_xy_error > float(self.p('start_xy_tolerance')):
                return None, f'start XY is {start_xy_error*1000:.1f}mm from target'
            if start_point[2] < float(self.p('start_min_z')):
                return None, f'start Z is too low: {start_point[2]:.4f}m'
        if self.p('require_start_pitch'):
            start_pitch = float(np.sum(start[1:4]))
            if not float(self.p('start_min_pitch')) <= start_pitch <= float(self.p('start_max_pitch')):
                return None, f'start pitch is outside range: {math.degrees(start_pitch):.2f}deg'
        return start, ''

    def on_confirm(self, _request, response):
        if self.active:
            response.success, response.message = False, 'approach already active'
            return response
        start, error = self.validate_request()
        if error:
            response.success, response.message = False, error
            return response
        target_xy = np.asarray([self.target.pose.position.x, self.target.pose.position.y])
        try:
            goal = self.plan(start, target_xy)
        except Exception as exception:
            response.success, response.message = False, f'planning failed: {exception}'
            return response
        final = self.fk(goal)
        path = self.path_positions(start, goal)
        xy_error = float(np.linalg.norm(final[:2] - target_xy))
        minimum_z = min(point[2] for point in path)
        pitch = float(np.sum(goal[1:4]))
        if xy_error > float(self.p('max_xy_error')):
            response.success, response.message = False, f'planned XY error too large: {xy_error*1000:.1f}mm'
            return response
        if minimum_z + 1e-9 < float(self.p('min_path_z')):
            response.success, response.message = False, f'planned path Z too low: {minimum_z:.4f}m'
            return response
        summary = (f'planned q={[round(value, 5) for value in goal]}; '
                   f'XY error={xy_error*1000:.2f}mm, final Z={final[2]:.4f}m, '
                   f'path min Z={minimum_z:.4f}m, pitch={math.degrees(pitch):.2f}deg')
        if self.p('dry_run'):
            self.report(f'DRY_RUN: {summary}')
            response.success, response.message = False, f'dry_run=true; {summary}'
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
        self.started = self.get_clock().now()
        self.stable_since = None
        self.active = True
        self.report(f'moving: {summary}')
        response.success, response.message = True, 'MoveJ XY approach started'
        return response

    def on_cancel(self, _request, response):
        was_active = self.active
        self.active = False
        response.success, response.message = was_active, 'cancelled' if was_active else 'already idle'
        self.report(response.message)
        return response

    def update(self):
        if not self.active:
            return
        now = self.get_clock().now()
        elapsed = (now - self.started).nanoseconds / 1e9
        errors = [abs(self.positions.get(name, math.inf) - value) for name, value in self.goal.items()]
        maximum = max(errors)
        actual = np.asarray([self.positions[name] for name in self.goal], dtype=float)
        actual_point = self.fk(actual)
        target_xy = np.asarray([
            self.target.pose.position.x, self.target.pose.position.y], dtype=float)
        actual_xy_error = float(np.linalg.norm(actual_point[:2] - target_xy))
        actual_z = float(actual_point[2])
        actual_target_z_error = abs(actual_z - float(self.p('target_z')))
        if elapsed > float(self.p('timeout')):
            self.active = False
            self.report(
                f'FAILED: timeout; maximum joint error={math.degrees(maximum):.2f}deg, '
                f'actual XY error={actual_xy_error*1000:.2f}mm, Z={actual_z:.4f}m')
            return
        if (maximum > float(self.p('joint_tolerance')) or
                actual_xy_error > float(self.p('max_xy_error')) or
                actual_z < float(self.p('min_final_z')) or
                (bool(self.p('require_actual_target_z')) and
                 actual_target_z_error > float(self.p('actual_target_z_tolerance')))):
            self.stable_since = None
            return
        if elapsed < float(self.p('minimum_completion_time')):
            self.stable_since = None
            return
        if self.stable_since is None:
            self.stable_since = now
            return
        if (now - self.stable_since).nanoseconds / 1e9 >= float(self.p('settle_time')):
            self.active = False
            self.report(
                f'COMPLETED: maximum joint error={math.degrees(maximum):.2f}deg, '
                f'actual XY error={actual_xy_error*1000:.2f}mm, Z={actual_z:.4f}m')


def main(args=None):
    rclpy.init(args=args)
    node = MoveJXyApproach()
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
