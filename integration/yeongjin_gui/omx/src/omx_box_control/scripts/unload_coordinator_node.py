#!/usr/bin/env python3
"""Feedback-gated reverse pick/place cycle for the defect-zone OMX."""

from functools import partial
import math
from pathlib import Path

from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.optimize import minimize
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class UnloadCoordinator(Node):
    """Pick the box from Beagle and place it 180 degrees behind the OMX."""

    IDLE = 'IDLE'
    WAIT_STAGING = 'WAIT_STAGING'
    WAIT_OPEN_FOR_PICK = 'WAIT_OPEN_FOR_PICK'
    WAIT_SOURCE_APPROACH = 'WAIT_SOURCE_APPROACH'
    WAIT_SOURCE_DESCENT = 'WAIT_SOURCE_DESCENT'
    WAIT_GRASP = 'WAIT_GRASP'
    WAIT_SOURCE_LIFT = 'WAIT_SOURCE_LIFT'
    WAIT_RETRY_LIFT = 'WAIT_RETRY_LIFT'
    WAIT_RETRY_OPEN = 'WAIT_RETRY_OPEN'
    WAIT_RETRY_DESCENT = 'WAIT_RETRY_DESCENT'
    WAIT_ROTATE_TO_DESTINATION = 'WAIT_ROTATE_TO_DESTINATION'
    WAIT_DESTINATION_DESCENT = 'WAIT_DESTINATION_DESCENT'
    WAIT_RELEASE = 'WAIT_RELEASE'
    WAIT_DESTINATION_RETRACT = 'WAIT_DESTINATION_RETRACT'
    WAIT_ROTATE_RETURN = 'WAIT_ROTATE_RETURN'
    WAIT_RETURN_STAGING = 'WAIT_RETURN_STAGING'
    WAIT_RETURN_PARKING = 'WAIT_RETURN_PARKING'
    WAIT_FINAL_CLOSE = 'WAIT_FINAL_GRIPPER_CLOSE'
    WAIT_ABORT_SOURCE_LIFT = 'WAIT_ABORT_SOURCE_LIFT'
    WAIT_ABORT_DESTINATION_LIFT = 'WAIT_ABORT_DESTINATION_LIFT'
    WAIT_ABORT_ROTATE_RETURN = 'WAIT_ABORT_ROTATE_RETURN'
    WAIT_ABORT_RETURN_STAGING = 'WAIT_ABORT_RETURN_STAGING'
    WAIT_ABORT_RETURN_PARKING = 'WAIT_ABORT_RETURN_PARKING'
    WAIT_ABORT_FINAL_CLOSE = 'WAIT_ABORT_FINAL_GRIPPER_CLOSE'
    PICK_FAILED_COMPLETE = 'PICK_FAILED_COMPLETE'
    COMPLETE = 'COMPLETE'
    FAILED = 'FAILED'

    # These motions run only after the box has been released, or while safely
    # returning from an aborted cycle. A bounded same-goal trim is safe here
    # and avoids turning a completed unload into a false terminal failure.
    RETURN_TRIM_STATES = {
        WAIT_DESTINATION_RETRACT, WAIT_ROTATE_RETURN,
        WAIT_RETURN_STAGING, WAIT_RETURN_PARKING,
        WAIT_ABORT_SOURCE_LIFT, WAIT_ABORT_DESTINATION_LIFT,
        WAIT_ABORT_ROTATE_RETURN, WAIT_ABORT_RETURN_STAGING,
        WAIT_ABORT_RETURN_PARKING,
    }

    def __init__(self):
        super().__init__('unload_coordinator')
        defaults = {
            'movej_topic': 'omx_movej_controller/movej',
            'joint_states_topic': 'joint_states',
            'gripper_action': 'gripper_controller/gripper_cmd',
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'],
            'gripper_joint_name': 'gripper_joint_1',
            'joint_lower': [-4.71239, -2.0944, -2.0944, -1.74533, -4.71239],
            'joint_upper': [6.28319, 1.5708, 1.5708, 1.74533, 4.71239],
            'joint_limit_margin': 0.06,
            'staging_positions': [0.008422, -0.387875, -0.181383, 1.639417, 0.0],
            'parking_positions': [0.067495, -2.009515, 1.713457, 1.647495, -0.067495],
            # The source X/Y and Z values reuse the validated loading OMX place pose.
            'source_xy': [0.20814601, -0.02140298],
            'teach_profile_file': '/root/omx_box_project_ws/integration/yeongjin_gui/runtime/calibration/unload_source_teach.yaml',
            'vision_target_topic': 'vision_target',
            'require_vision_target': True,
            'vision_target_timeout': 2.0,
            'vision_target_frame': 'link0',
            'maximum_vision_offset': 0.10,
            'minimum_target_radius': 0.10,
            'maximum_target_radius': 0.29,
            'source_approach_z': 0.19253,
            'source_grasp_z': 0.14099,
            'destination_rotation': math.pi,
            'destination_release_z': 0.022,
            'joint5_position': 0.0,
            'minimum_pitch': 1.047197551,
            'maximum_pitch': 1.483529864,
            'preferred_pitch': 1.221730476,
            'fk_position_bias': [0.00126, 0.0, 0.00055],
            'path_samples': 101,
            'minimum_transfer_path_z': 0.110,
            'minimum_source_vertical_path_z': 0.140,
            'minimum_destination_vertical_path_z': 0.000,
            'maximum_ik_error': 0.006,
            'move_duration': 13.0,
            # Unload-only controller caps permit this faster 180-degree move.
            'rotate_duration': 19.0,
            'vertical_duration': 3.0,
            'destination_vertical_duration': 10.0,
            'staging_duration': 13.0,
            'return_staging_duration': 9.0,
            'parking_duration': 12.0,
            'maximum_start_joint_delta': 2.0,
            'joint_tolerance': 0.060,
            'settle_time': 0.15,
            'move_timeout_margin': 5.0,
            'return_trim_retry_limit': 1,
            'return_trim_duration': 6.0,
            'parking_completion_tolerance': 0.18,
            'feedback_timeout': 0.5,
            'gripper_open_position': 0.65,
            'gripper_closed_position': 0.0,
            'gripper_effort': 10.0,
            'minimum_open_position': 0.53,
            'min_grasp_position': 0.15,
            'max_grasp_position': 0.70,
            'maximum_final_closed_position': 0.05,
            'max_pick_retries': 2,
            'gripper_watchdog_timeout': 2.5,
            'gripper_watchdog_stable_time': 0.50,
            'gripper_position_epsilon': 0.003,
            'dry_run': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.source_xy = np.asarray(self.p('source_xy'), dtype=float)
        self.load_teach_profile()

        self.state = self.IDLE
        self.positions = {}
        self.feedback_time = None
        self.goals = {}
        self.motion_goal = None
        self.motion_started = None
        self.motion_duration = 0.0
        self.motion_stable_since = None
        self.motion_retry_count = 0
        self.gripper_goal_handle = None
        self.gripper_started = None
        self.gripper_stable_since = None
        self.gripper_last_position = None
        self.gripper_opening = False
        self.gripper_watchdog_cancel = False
        self.vision_xy = None
        self.vision_joint5 = None
        self.vision_target_time = None
        self.cycle_xy = None
        self.cycle_joint5 = None
        self.pick_retry_count = 0
        self.return_mode = None
        self.failed_from_state = None

        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.move_pub = self.create_publisher(
            JointTrajectory, str(self.p('movej_topic')), 10)
        self.create_subscription(
            JointState, str(self.p('joint_states_topic')), self.on_joints, 10)
        self.create_subscription(
            PoseStamped, str(self.p('vision_target_topic')), self.on_vision_target, 10)
        self.gripper = ActionClient(
            self, GripperCommand, str(self.p('gripper_action')))
        self.create_service(Trigger, '~/start', self.on_start)
        self.create_service(Trigger, '~/cancel', self.on_cancel)
        self.create_service(Trigger, '~/reset', self.on_reset)
        self.create_timer(0.05, self.update)
        self.report('ready; call ~/start after Beagle reaches the defect zone')

    def p(self, name):
        return self.get_parameter(name).value

    def load_teach_profile(self):
        path = Path(str(self.p('teach_profile_file')))
        if not path.is_file():
            self.get_logger().warning(
                f'unload source teach profile is absent: {path}; '
                'automatic vision target remains unavailable until teaching')
            return
        try:
            import yaml
            data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
            source = np.asarray(data['taught_source_xy'], dtype=float)
            if source.shape != (2,) or not np.all(np.isfinite(source)):
                raise ValueError('taught_source_xy must contain two finite values')
            self.source_xy = source
            self.get_logger().info(
                f'loaded taught unload source XY: {self.source_xy.tolist()}')
        except Exception as error:  # noqa: BLE001 - configuration boundary
            self.get_logger().error(f'cannot load unload source teach profile {path}: {error}')

    def report(self, text):
        message = String(data=f'{self.state}: {text}')
        self.status_pub.publish(message)
        self.get_logger().info(message.data)

    def enter(self, state, text):
        self.state = state
        self.report(text)

    def fail(self, reason):
        self.failed_from_state = self.state
        self.motion_goal = None
        self.enter(self.FAILED, f'{reason}; no further command sent')

    def on_joints(self, message):
        self.positions.update(zip(message.name, message.position))
        self.feedback_time = self.get_clock().now()

    def on_vision_target(self, message):
        expected_frame = str(self.p('vision_target_frame'))
        if message.header.frame_id and message.header.frame_id != expected_frame:
            self.get_logger().warning(
                f'ignored unload target frame={message.header.frame_id}; '
                f'expected={expected_frame}')
            return
        xy = np.asarray([message.pose.position.x, message.pose.position.y], dtype=float)
        nominal = self.source_xy
        center = np.asarray([-0.00999, 0.0])
        radius = float(np.linalg.norm(xy - center))
        if (not np.all(np.isfinite(xy)) or
                float(np.linalg.norm(xy - nominal)) > float(self.p('maximum_vision_offset')) or
                not float(self.p('minimum_target_radius')) <= radius <=
                    float(self.p('maximum_target_radius'))):
            self.get_logger().warning(
                f'ignored unsafe unload vision target: x={xy[0]:.4f}, y={xy[1]:.4f}')
            return
        quaternion = message.pose.orientation
        joint5 = 2.0 * math.atan2(float(quaternion.x), float(quaternion.w))
        if not math.isfinite(joint5):
            self.get_logger().warning('ignored unload target with invalid joint5')
            return
        self.vision_xy = xy
        self.vision_joint5 = joint5
        self.vision_target_time = self.get_clock().now()

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

    def solve_pose(self, seed, target, joint5=None):
        lower = np.asarray(self.p('joint_lower'), dtype=float)
        upper = np.asarray(self.p('joint_upper'), dtype=float)
        margin = float(self.p('joint_limit_margin'))
        q5 = float(self.p('joint5_position') if joint5 is None else joint5)
        target = np.asarray(target, dtype=float)

        bias = np.asarray(self.p('fk_position_bias'), dtype=float)
        radial_x = target[0] + 0.01125 - bias[0]
        radial_y = target[1] - bias[1]
        radial_distance = math.hypot(radial_x, radial_y)
        lateral = -0.0016 * math.cos(q5)
        if radial_distance <= abs(lateral):
            raise RuntimeError('source target is too close to the joint1 axis')
        local_x = math.sqrt(radial_distance ** 2 - lateral ** 2)
        q1 = math.atan2(radial_y, radial_x) - math.atan2(lateral, local_x)
        if not lower[0] + margin <= q1 <= upper[0] - margin:
            raise RuntimeError('source joint1 violates its safety margin')

        bounds = list(zip(lower[1:4] + margin, upper[1:4] - margin))

        def full(values):
            return np.r_[q1, values, q5]

        def objective(values):
            goal = full(values)
            error = self.fk(goal) - target
            pitch_error = float(np.sum(goal[1:4])) - float(self.p('preferred_pitch'))
            delta = goal[1:4] - seed[1:4]
            return (30000.0 * float(error @ error) +
                    2.0 * pitch_error ** 2 + 0.05 * float(delta @ delta))

        def constraints(values):
            goal = full(values)
            pitch = float(np.sum(goal[1:4]))
            return np.asarray([
                pitch - float(self.p('minimum_pitch')),
                float(self.p('maximum_pitch')) - pitch,
            ])

        candidates = []
        for values in (
                seed[1:4], np.asarray([-0.3, 0.3, 1.0]),
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
            raise RuntimeError(f'IK error is too large: {error*1000:.2f}mm')
        return goal

    def rotate_goal(self, goal, angle):
        rotated = np.asarray(goal, dtype=float).copy()
        rotated[0] += float(angle)
        lower = float(self.p('joint_lower')[0]) + float(self.p('joint_limit_margin'))
        upper = float(self.p('joint_upper')[0]) - float(self.p('joint_limit_margin'))
        while rotated[0] > upper:
            rotated[0] -= 2.0 * math.pi
        while rotated[0] < lower:
            rotated[0] += 2.0 * math.pi
        if not lower <= rotated[0] <= upper:
            raise RuntimeError('180-degree destination violates joint1 margin')
        return rotated

    def validate_path(self, start, goal, minimum_z):
        samples = max(2, int(self.p('path_samples')))
        path = [self.fk(start + alpha * (goal - start))
                for alpha in np.linspace(0.0, 1.0, samples)]
        path_minimum = min(float(point[2]) for point in path)
        if path_minimum < minimum_z:
            raise RuntimeError(
                f'planned path Z={path_minimum:.4f}m below limit={minimum_z:.4f}m')

    def plan_cycle(self):
        staging = np.asarray(self.p('staging_positions'), dtype=float)
        if bool(self.p('require_vision_target')):
            xy = np.asarray(self.vision_xy, dtype=float)
            joint5 = float(self.vision_joint5)
        else:
            xy = self.source_xy.copy()
            joint5 = float(self.p('joint5_position'))
        self.cycle_xy = xy.copy()
        self.cycle_joint5 = joint5
        source_approach = self.solve_pose(
            staging, [xy[0], xy[1], float(self.p('source_approach_z'))], joint5)
        source_grasp = self.solve_pose(
            source_approach, [xy[0], xy[1], float(self.p('source_grasp_z'))], joint5)
        angle = float(self.p('destination_rotation'))
        destination_approach = self.rotate_goal(source_approach, angle)
        destination_xyz = self.fk(destination_approach)
        destination_release = self.solve_pose(
            destination_approach,
            [destination_xyz[0], destination_xyz[1],
             float(self.p('destination_release_z'))],
            joint5)

        self.validate_path(
            staging, source_approach,
            float(self.p('minimum_transfer_path_z')))
        self.validate_path(
            source_approach, source_grasp,
            float(self.p('minimum_source_vertical_path_z')))
        self.validate_path(
            source_approach, destination_approach,
            float(self.p('source_approach_z')) - 0.010)
        self.validate_path(
            destination_approach, destination_release,
            float(self.p('minimum_destination_vertical_path_z')))
        self.goals = {
            'parking': np.asarray(self.p('parking_positions'), dtype=float),
            'staging': staging,
            'source_approach': source_approach,
            'source_grasp': source_grasp,
            'destination_approach': destination_approach,
            'destination_release': destination_release,
        }

    def preflight_error(self):
        names = list(self.p('joint_names')) + [str(self.p('gripper_joint_name'))]
        if self.feedback_time is None or not all(name in self.positions for name in names):
            return 'unload OMX joint or gripper feedback is unavailable'
        age = (self.get_clock().now() - self.feedback_time).nanoseconds / 1e9
        if age > float(self.p('feedback_timeout')):
            return f'unload OMX joint feedback is stale ({age:.2f}s)'
        if not self.gripper.server_is_ready():
            return 'unload OMX gripper action is unavailable'
        if bool(self.p('require_vision_target')):
            if (self.vision_target_time is None or self.vision_xy is None or
                    self.vision_joint5 is None):
                return 'stable unload camera target is unavailable'
            target_age = (
                self.get_clock().now() - self.vision_target_time).nanoseconds / 1e9
            if target_age > float(self.p('vision_target_timeout')):
                return f'unload camera target is stale ({target_age:.2f}s)'
        return ''

    def on_start(self, _request, response):
        if self.state not in (self.IDLE, self.COMPLETE, self.FAILED):
            response.success, response.message = False, f'busy: {self.state}'
            return response
        error = self.preflight_error()
        if error:
            response.success, response.message = False, error
            return response
        try:
            self.plan_cycle()
        except Exception as exception:  # noqa: BLE001 - planner boundary
            response.success, response.message = False, f'unload planning failed: {exception}'
            self.enter(self.FAILED, response.message)
            return response
        self.pick_retry_count = 0
        self.return_mode = None
        self.failed_from_state = None
        names = list(self.p('joint_names'))
        current = np.asarray([self.positions[name] for name in names], dtype=float)
        start_delta = float(np.max(np.abs(current - self.goals['staging'])))
        if start_delta > float(self.p('maximum_start_joint_delta')):
            response.success, response.message = False, (
                f'unload OMX is too far from staging: '
                f'{math.degrees(start_delta):.1f}deg')
            self.enter(self.FAILED, response.message)
            return response
        if bool(self.p('dry_run')):
            summary = ', '.join(
                f'{name}={[round(float(value), 4) for value in goal]}'
                for name, goal in self.goals.items())
            response.success, response.message = True, f'dry-run planning passed; {summary}'
            self.report(response.message)
            return response
        self.command_motion(
            self.goals['staging'], self.WAIT_STAGING,
            float(self.p('staging_duration')), 'move to unloading staging')
        response.success, response.message = True, (
            f'automatic OMX unload started; target=({self.cycle_xy[0]:.4f}, '
            f'{self.cycle_xy[1]:.4f}), joint5={self.cycle_joint5:.4f}')
        return response

    def on_cancel(self, _request, response):
        active = self.state not in (self.IDLE, self.COMPLETE, self.FAILED)
        if self.gripper_goal_handle is not None:
            self.gripper_goal_handle.cancel_goal_async()
        names = list(self.p('joint_names'))
        if all(name in self.positions for name in names):
            self.publish_trajectory(
                np.asarray([self.positions[name] for name in names]), 0.1)
        self.motion_goal = None
        self.return_mode = None
        self.enter(self.IDLE, 'unload cycle cancelled; current arm pose held')
        response.success, response.message = active, 'cancelled' if active else 'already idle'
        return response

    def on_reset(self, _request, response):
        """Return the unloading arm to parking without sweeping from a low pose."""
        names = list(self.p('joint_names'))
        if self.feedback_time is None or not all(name in self.positions for name in names):
            response.success, response.message = False, 'unload reset: joint feedback unavailable'
            return response
        if self.gripper_goal_handle is not None:
            self.gripper_goal_handle.cancel_goal_async()
            self.gripper_goal_handle = None
        self.motion_goal = None
        self.return_mode = 'reset'
        origin = self.failed_from_state if self.state == self.FAILED else self.state
        source_low = {
            self.WAIT_SOURCE_DESCENT, self.WAIT_GRASP, self.WAIT_RETRY_DESCENT}
        destination_low = {self.WAIT_DESTINATION_DESCENT, self.WAIT_RELEASE}
        staging = self.goals.get(
            'staging', np.asarray(self.p('staging_positions'), dtype=float))
        parking = self.goals.get(
            'parking', np.asarray(self.p('parking_positions'), dtype=float))
        self.goals['staging'] = np.asarray(staging, dtype=float)
        self.goals['parking'] = np.asarray(parking, dtype=float)
        if origin in source_low and 'source_approach' in self.goals:
            self.command_motion(
                self.goals['source_approach'], self.WAIT_ABORT_SOURCE_LIFT,
                float(self.p('vertical_duration')), 'reset lift from Beagle box')
        elif origin in destination_low and 'destination_approach' in self.goals:
            self.command_motion(
                self.goals['destination_approach'], self.WAIT_ABORT_DESTINATION_LIFT,
                float(self.p('vertical_duration')), 'reset lift from unload destination')
        elif origin in (self.IDLE, self.COMPLETE, self.PICK_FAILED_COMPLETE):
            self.command_motion(
                self.goals['parking'], self.WAIT_ABORT_RETURN_PARKING,
                float(self.p('parking_duration')), 'reset unloading OMX to parking')
        else:
            self.command_motion(
                self.goals['staging'], self.WAIT_ABORT_RETURN_STAGING,
                float(self.p('return_staging_duration')), 'reset unloading OMX to staging')
        response.success, response.message = True, 'unload OMX reset return started'
        return response

    def publish_trajectory(self, goal, duration):
        message = JointTrajectory(joint_names=list(self.p('joint_names')))
        point = JointTrajectoryPoint(positions=[float(value) for value in goal])
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1.0e9)
        message.points = [point]
        self.move_pub.publish(message)

    def command_motion(self, goal, state, duration, description, retry=False):
        if not retry:
            self.motion_retry_count = 0
        self.motion_goal = np.asarray(goal, dtype=float)
        self.motion_duration = max(0.02, float(duration))
        self.motion_started = self.get_clock().now()
        self.motion_stable_since = None
        self.publish_trajectory(self.motion_goal, self.motion_duration)
        self.enter(state, f'{description} started; duration={self.motion_duration:.1f}s')

    def command_gripper(self, opening, state, description):
        self.gripper_opening = opening
        self.gripper_goal_handle = None
        self.gripper_started = self.get_clock().now()
        self.gripper_stable_since = None
        self.gripper_last_position = self.positions.get(str(self.p('gripper_joint_name')))
        self.gripper_watchdog_cancel = False
        goal = GripperCommand.Goal()
        goal.command.position = float(self.p(
            'gripper_open_position' if opening else 'gripper_closed_position'))
        goal.command.max_effort = float(self.p('gripper_effort'))
        future = self.gripper.send_goal_async(goal)
        future.add_done_callback(partial(self.on_gripper_goal, description))
        self.enter(state, f'{description} requested')

    def on_gripper_goal(self, description, future):
        try:
            handle = future.result()
        except Exception as exception:  # noqa: BLE001 - ROS future boundary
            reason = f'{description} goal error: {exception}'
            if self.state == self.WAIT_GRASP:
                self.handle_grasp_failure(reason)
            else:
                self.fail(reason)
            return
        if not handle.accepted:
            reason = f'{description} goal rejected'
            if self.state == self.WAIT_GRASP:
                self.handle_grasp_failure(reason)
            else:
                self.fail(reason)
            return
        self.gripper_goal_handle = handle
        handle.get_result_async().add_done_callback(
            partial(self.on_gripper_result, description))

    def valid_gripper_position(self, position):
        if self.gripper_opening:
            return position >= float(self.p('minimum_open_position'))
        if self.state in (self.WAIT_FINAL_CLOSE, self.WAIT_ABORT_FINAL_CLOSE):
            return position <= float(self.p('maximum_final_closed_position'))
        return (float(self.p('min_grasp_position')) <= position <=
                float(self.p('max_grasp_position')))

    def on_gripper_result(self, description, future):
        if self.state not in (
                self.WAIT_OPEN_FOR_PICK, self.WAIT_GRASP, self.WAIT_RELEASE,
                self.WAIT_RETRY_OPEN, self.WAIT_FINAL_CLOSE,
                self.WAIT_ABORT_FINAL_CLOSE):
            return
        try:
            wrapped = future.result()
        except Exception as exception:  # noqa: BLE001 - ROS future boundary
            self.fail(f'{description} result error: {exception}')
            return
        position = self.positions.get(str(self.p('gripper_joint_name')))
        accepted_terminal = wrapped.status == GoalStatus.STATUS_SUCCEEDED
        accepted_stall = (
            wrapped.status == GoalStatus.STATUS_ABORTED and bool(wrapped.result.stalled))
        accepted_watchdog = (
            wrapped.status == GoalStatus.STATUS_CANCELED and self.gripper_watchdog_cancel)
        self.gripper_goal_handle = None
        if (not (accepted_terminal or accepted_stall or accepted_watchdog) or
                position is None or not self.valid_gripper_position(float(position))):
            reason = f'{description} failed: status={wrapped.status}, position={position}'
            if self.state == self.WAIT_GRASP:
                self.handle_grasp_failure(reason)
            else:
                self.fail(reason)
            return
        self.after_gripper_complete()

    def handle_grasp_failure(self, reason):
        """Lift clear before retrying, or park safely after retry exhaustion."""
        self.gripper_goal_handle = None
        maximum = max(0, int(self.p('max_pick_retries')))
        if self.pick_retry_count < maximum:
            self.pick_retry_count += 1
            self.command_motion(
                self.goals['source_approach'], self.WAIT_RETRY_LIFT,
                float(self.p('vertical_duration')),
                f'unload grasp failed; lift for retry {self.pick_retry_count}/{maximum}')
            return
        self.return_mode = 'pick_failed'
        self.command_motion(
            self.goals['source_approach'], self.WAIT_ABORT_SOURCE_LIFT,
            float(self.p('vertical_duration')),
            f'unload grasp failed after {maximum} retries; lift before parking: {reason}')

    def after_gripper_complete(self):
        if self.state == self.WAIT_OPEN_FOR_PICK:
            self.command_motion(
                self.goals['source_approach'], self.WAIT_SOURCE_APPROACH,
                float(self.p('move_duration')), 'approach Beagle box')
        elif self.state == self.WAIT_GRASP:
            self.command_motion(
                self.goals['source_approach'], self.WAIT_SOURCE_LIFT,
                float(self.p('vertical_duration')), 'lift box from Beagle')
        elif self.state == self.WAIT_RELEASE:
            self.command_motion(
                self.goals['destination_approach'], self.WAIT_DESTINATION_RETRACT,
                float(self.p('vertical_duration')), 'retract after unloading')
        elif self.state == self.WAIT_RETRY_OPEN:
            self.command_motion(
                self.goals['source_grasp'], self.WAIT_RETRY_DESCENT,
                float(self.p('vertical_duration')),
                f'descend for unload grasp retry {self.pick_retry_count}')
        elif self.state == self.WAIT_FINAL_CLOSE:
            self.enter(self.COMPLETE, 'automatic OMX unloading complete; gripper closed')
        elif self.state == self.WAIT_ABORT_FINAL_CLOSE:
            if self.return_mode == 'pick_failed':
                self.enter(
                    self.PICK_FAILED_COMPLETE,
                    f'unload pick failed after {self.pick_retry_count} retries; parked safely')
            else:
                self.return_mode = None
                self.failed_from_state = None
                self.enter(self.IDLE, 'unload OMX reset complete; parking ready')

    def after_motion_complete(self):
        if self.state == self.WAIT_STAGING:
            self.command_gripper(True, self.WAIT_OPEN_FOR_PICK, 'open before unload pick')
        elif self.state == self.WAIT_SOURCE_APPROACH:
            self.command_motion(
                self.goals['source_grasp'], self.WAIT_SOURCE_DESCENT,
                float(self.p('vertical_duration')), 'descend to Beagle box')
        elif self.state == self.WAIT_SOURCE_DESCENT:
            self.command_gripper(False, self.WAIT_GRASP, 'grasp Beagle box')
        elif self.state == self.WAIT_SOURCE_LIFT:
            self.command_motion(
                self.goals['destination_approach'], self.WAIT_ROTATE_TO_DESTINATION,
                float(self.p('rotate_duration')), 'rotate loaded box 180 degrees')
        elif self.state == self.WAIT_RETRY_LIFT:
            self.command_gripper(
                True, self.WAIT_RETRY_OPEN,
                f'open before unload grasp retry {self.pick_retry_count}')
        elif self.state == self.WAIT_RETRY_DESCENT:
            self.command_gripper(
                False, self.WAIT_GRASP,
                f'unload grasp retry {self.pick_retry_count}')
        elif self.state == self.WAIT_ROTATE_TO_DESTINATION:
            self.command_motion(
                self.goals['destination_release'], self.WAIT_DESTINATION_DESCENT,
                float(self.p('destination_vertical_duration')),
                'descend to unload destination')
        elif self.state == self.WAIT_DESTINATION_DESCENT:
            self.command_gripper(True, self.WAIT_RELEASE, 'release unloaded box')
        elif self.state == self.WAIT_DESTINATION_RETRACT:
            self.command_motion(
                self.goals['source_approach'], self.WAIT_ROTATE_RETURN,
                float(self.p('rotate_duration')), 'rotate empty gripper back')
        elif self.state == self.WAIT_ROTATE_RETURN:
            self.command_motion(
                self.goals['staging'], self.WAIT_RETURN_STAGING,
                float(self.p('return_staging_duration')), 'return unloading staging')
        elif self.state == self.WAIT_RETURN_STAGING:
            self.command_motion(
                self.goals['parking'], self.WAIT_RETURN_PARKING,
                float(self.p('parking_duration')), 'clear camera view for next Beagle')
        elif self.state == self.WAIT_RETURN_PARKING:
            self.motion_goal = None
            self.command_gripper(
                False, self.WAIT_FINAL_CLOSE, 'close gripper after unloading')
        elif self.state == self.WAIT_ABORT_SOURCE_LIFT:
            self.command_motion(
                self.goals['staging'], self.WAIT_ABORT_RETURN_STAGING,
                float(self.p('return_staging_duration')),
                'return unloading OMX to staging after source clearance')
        elif self.state == self.WAIT_ABORT_DESTINATION_LIFT:
            self.command_motion(
                self.goals['source_approach'], self.WAIT_ABORT_ROTATE_RETURN,
                float(self.p('rotate_duration')),
                'rotate unloading OMX back after destination clearance')
        elif self.state == self.WAIT_ABORT_ROTATE_RETURN:
            self.command_motion(
                self.goals['staging'], self.WAIT_ABORT_RETURN_STAGING,
                float(self.p('return_staging_duration')),
                'return unloading OMX to staging during reset')
        elif self.state == self.WAIT_ABORT_RETURN_STAGING:
            self.command_motion(
                self.goals['parking'], self.WAIT_ABORT_RETURN_PARKING,
                float(self.p('parking_duration')),
                'return unloading OMX to parking')
        elif self.state == self.WAIT_ABORT_RETURN_PARKING:
            self.motion_goal = None
            self.command_gripper(
                False, self.WAIT_ABORT_FINAL_CLOSE,
                'close gripper after unload abort/reset return')

    def update_gripper_watchdog(self, now):
        if self.gripper_goal_handle is None or self.gripper_started is None:
            return
        position = self.positions.get(str(self.p('gripper_joint_name')))
        if position is None:
            return
        epsilon = float(self.p('gripper_position_epsilon'))
        if (self.gripper_last_position is None or
                abs(float(position) - float(self.gripper_last_position)) > epsilon):
            self.gripper_last_position = float(position)
            self.gripper_stable_since = now
            return
        if self.gripper_stable_since is None:
            self.gripper_stable_since = now
            return
        elapsed = (now - self.gripper_started).nanoseconds / 1e9
        stable = (now - self.gripper_stable_since).nanoseconds / 1e9
        watchdog_ready = (
            elapsed >= float(self.p('gripper_watchdog_timeout')) and
            stable >= float(self.p('gripper_watchdog_stable_time')) and
            not self.gripper_watchdog_cancel)
        if watchdog_ready and self.valid_gripper_position(float(position)):
            self.gripper_watchdog_cancel = True
            self.report(
                f'accepted stable gripper={float(position):.4f}rad; '
                'cancelling non-terminating action')
            self.gripper_goal_handle.cancel_goal_async()
        elif watchdog_ready and self.state == self.WAIT_GRASP:
            # An empty close can stall forever without an action result. Cancel
            # it so on_gripper_result enters the bounded lift-and-retry path.
            self.gripper_watchdog_cancel = True
            self.report(
                f'empty grasp suspected at gripper={float(position):.4f}rad; '
                'cancelling action for retry')
            self.gripper_goal_handle.cancel_goal_async()

    def update(self):
        now = self.get_clock().now()
        self.update_gripper_watchdog(now)
        if self.motion_goal is None or self.motion_started is None:
            return
        if self.feedback_time is None or (
                now - self.feedback_time).nanoseconds / 1e9 > float(self.p('feedback_timeout')):
            self.fail('joint feedback disappeared during unload motion')
            return
        names = list(self.p('joint_names'))
        if not all(name in self.positions for name in names):
            self.fail('joint feedback became incomplete during unload motion')
            return
        actual = np.asarray([self.positions[name] for name in names], dtype=float)
        maximum_error = float(np.max(np.abs(actual - self.motion_goal)))
        elapsed = (now - self.motion_started).nanoseconds / 1e9
        if self.state in (
                self.WAIT_RETURN_PARKING, self.WAIT_ABORT_RETURN_PARKING
                ) and elapsed >= self.motion_duration:
            vision_fresh = (self.vision_target_time is not None and
                            (now - self.vision_target_time).nanoseconds / 1e9 <=
                            float(self.p('vision_target_timeout')))
            if vision_fresh:
                self.report(
                    f'vision parking accepted with camera clear; remaining joint error='
                    f'{math.degrees(maximum_error):.2f}deg')
                self.motion_goal = None
                self.after_motion_complete()
                return
        timeout = self.motion_duration + float(self.p('move_timeout_margin'))
        if elapsed > timeout:
            retry_limit = max(0, int(self.p('return_trim_retry_limit')))
            if (self.state in self.RETURN_TRIM_STATES and
                    self.motion_retry_count < retry_limit):
                state = self.state
                goal = self.motion_goal.copy()
                self.motion_retry_count += 1
                retry_number = self.motion_retry_count
                self.command_motion(
                    goal, state, float(self.p('return_trim_duration')),
                    f'trim return pose retry {retry_number}/{retry_limit}; '
                    f'previous maximum joint error={math.degrees(maximum_error):.2f}deg',
                    retry=True)
                return
            if (self.state in (
                    self.WAIT_RETURN_PARKING, self.WAIT_ABORT_RETURN_PARKING) and
                    maximum_error <= float(self.p('parking_completion_tolerance'))):
                self.report(
                    f'parking accepted after bounded trim; maximum joint error='
                    f'{math.degrees(maximum_error):.2f}deg; closing gripper')
                self.motion_goal = None
                self.after_motion_complete()
                return
            self.fail(
                f'motion timeout in {self.state}; maximum joint error='
                f'{math.degrees(maximum_error):.2f}deg')
            return
        if (elapsed < self.motion_duration or
                maximum_error > float(self.p('joint_tolerance'))):
            self.motion_stable_since = None
            return
        if self.motion_stable_since is None:
            self.motion_stable_since = now
            return
        if (now - self.motion_stable_since).nanoseconds / 1e9 < float(self.p('settle_time')):
            return
        self.report(
            f'motion completed; maximum joint error='
            f'{math.degrees(maximum_error):.2f}deg, Z={self.fk(actual)[2]:.4f}m')
        self.motion_goal = None
        self.after_motion_complete()


def main(args=None):
    rclpy.init(args=args)
    node = UnloadCoordinator()
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
