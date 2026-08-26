#!/usr/bin/env python3
"""Feedback-gated MoveJ descent into a safe Cartesian Z acceptance band."""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from movej_xy_approach_node import MoveJXyApproach


class MoveJClosedLoopDescent(MoveJXyApproach):
    def __init__(self):
        self.command_target_z = None
        self.command_preferred_pitch = None
        super().__init__()
        extras = {
            'target_z_min': 0.080,
            'target_z_max': 0.095,
            'planned_step': 0.001,
            'max_steps': 8,
            'max_actual_step': 0.015,
            'minimum_actual_progress': 0.0002,
            'max_pitch_step': 0.087266463,
            'fast_above_target_margin': 0.025,
            'fast_move_duration': 3.5,
            'fast_minimum_completion_time': 4.5,
            'single_step_mode': False,
            'cartesian_completion_enabled': False,
            'gripper_joint_name': 'gripper_joint_1',
            'minimum_open_position': 0.80,
        }
        for name, value in extras.items():
            self.declare_parameter(name, value)
        self.initial_z = self.final_target_z = self.last_start_z = None
        self.last_start_pitch = None
        self.step_minimum_completion_time = None
        self.step_index = 0

    def p(self, name):
        if name == 'target_z' and self.command_target_z is not None:
            return self.command_target_z
        if name == 'preferred_pitch' and self.command_preferred_pitch is not None:
            return self.command_preferred_pitch
        return super().p(name)

    def current_joints(self):
        names = list(self.p('joint_names'))
        if not all(name in self.positions for name in names):
            return None
        return np.asarray([self.positions[name] for name in names], dtype=float)

    def publish_step(self, start):
        current = self.fk(start)
        remaining = current[2] - self.final_target_z
        if remaining <= 0.0:
            raise RuntimeError('measured Z is already at or below the band upper edge')
        step = min(float(self.p('planned_step')), remaining)
        self.command_target_z = current[2] - step
        # Preserve the measured pitch for this step.  The previous fixed
        # preferred pitch caused joint3/joint4 counter-rotation whose tracking
        # mismatch amplified a 1mm Cartesian request into a 17-20mm drop.
        self.command_preferred_pitch = float(np.sum(start[1:4]))
        target_xy = np.asarray([self.target.pose.position.x, self.target.pose.position.y])
        goal = self.plan(start, target_xy)
        final = self.fk(goal)
        path_min = min(point[2] for point in self.path_positions(start, goal))
        if path_min + 1e-9 < self.command_target_z - float(self.p('z_tolerance')):
            raise RuntimeError(f'path minimum {path_min:.4f}m is below step target')
        message = JointTrajectory()
        message.joint_names = list(self.p('joint_names'))
        point = JointTrajectoryPoint()
        point.positions = goal.tolist()
        fast_profile = (
            current[2] > self.final_target_z +
            float(self.p('fast_above_target_margin')))
        duration = max(0.02, float(self.p(
            'fast_move_duration' if fast_profile else 'move_duration')))
        self.step_minimum_completion_time = float(self.p(
            'fast_minimum_completion_time' if fast_profile else
            'minimum_completion_time'))
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1.0e9)
        message.points = [point]
        self.move_pub.publish(message)
        self.goal = dict(zip(message.joint_names, goal))
        self.last_start_z = current[2]
        self.last_start_pitch = float(np.sum(start[1:4]))
        self.started = self.get_clock().now()
        self.stable_since = None
        self.step_index += 1
        self.report(
            f'step {self.step_index}: measured Z={current[2]:.4f}m -> '
            f'planned Z={final[2]:.4f}m, path min={path_min:.4f}m, '
            f'pitch={math.degrees(float(np.sum(goal[1:4]))):.2f}deg, '
            f'profile={"fast" if fast_profile else "slow"}, '
            f'duration={duration:.1f}s, '
            f'validation={self.step_minimum_completion_time:.1f}s')

    def on_confirm(self, _request, response):
        if self.active:
            response.success, response.message = False, 'descent already active'
            return response
        start, error = self.validate_request()
        if error:
            response.success, response.message = False, error
            return response
        gripper_name = self.p('gripper_joint_name')
        if gripper_name not in self.positions:
            response.success, response.message = False, 'gripper feedback unavailable'
            return response
        gripper_position = float(self.positions[gripper_name])
        if gripper_position < float(self.p('minimum_open_position')):
            response.success, response.message = False, (
                f'gripper is not fully open: {gripper_position:.4f}rad')
            return response
        z_min = float(self.p('target_z_min'))
        z_max = float(self.p('target_z_max'))
        step = float(self.p('planned_step'))
        if not float(self.p('min_final_z')) <= z_min < z_max:
            response.success, response.message = False, 'invalid target Z band'
            return response
        if step <= 0.0 or int(self.p('max_steps')) < 1:
            response.success, response.message = False, 'invalid planned_step/max_steps'
            return response
        initial = self.fk(start)
        self.initial_z = initial[2]
        self.final_target_z = z_max
        if self.initial_z < z_min:
            response.success, response.message = False, (
                f'start Z={self.initial_z:.4f}m is below safe band')
            return response
        if self.initial_z <= z_max:
            response.success, response.message = True, (
                f'already grasp-ready; Z={self.initial_z:.4f}m is in '
                f'[{z_min:.4f}, {z_max:.4f}]m')
            self.report(response.message)
            return response
        if self.p('dry_run'):
            old_target = self.command_target_z
            first_step = min(step, self.initial_z - z_max)
            self.command_target_z = self.initial_z - first_step
            target_xy = np.asarray([
                self.target.pose.position.x, self.target.pose.position.y])
            try:
                goal = self.plan(start, target_xy)
            except Exception as exception:
                response.success, response.message = False, f'dry-run planning failed: {exception}'
                self.command_target_z = old_target
                return response
            final = self.fk(goal)
            path_min = min(point[2] for point in self.path_positions(start, goal))
            xy_error = float(np.linalg.norm(final[:2] - target_xy))
            joint_delta = np.degrees(goal - start)
            self.command_target_z = old_target
            response.success = False
            response.message = (
                f'dry_run=true; measured Z={self.initial_z:.4f}m, '
                f'first-step Z={final[2]:.4f}m, path min={path_min:.4f}m, '
                f'XY error={xy_error*1000:.2f}mm, pitch='
                f'{math.degrees(float(np.sum(goal[1:4]))):.2f}deg, '
                f'joint delta deg={[round(value, 3) for value in joint_delta]}')
            self.report(response.message)
            return response
        self.step_index = 0
        self.active = True
        try:
            self.publish_step(start)
        except Exception as exception:
            self.active = False
            response.success, response.message = False, f'first step failed: {exception}'
            return response
        response.success, response.message = True, 'closed-loop MoveJ descent started'
        return response

    def update(self):
        if not self.active:
            return
        now = self.get_clock().now()
        elapsed = (now - self.started).nanoseconds / 1e9
        errors = [abs(self.positions.get(name, math.inf) - value)
                  for name, value in self.goal.items()]
        maximum = max(errors)
        if elapsed > float(self.p('timeout')):
            self.active = False
            self.report(f'FAILED: step timeout; joint error={math.degrees(maximum):.2f}deg')
            return
        minimum_completion_time = (
            self.step_minimum_completion_time
            if self.step_minimum_completion_time is not None else
            float(self.p('minimum_completion_time')))
        if elapsed < minimum_completion_time:
            self.stable_since = None
            return
        # In single-step validation, the measured Cartesian result is the goal;
        # joint-space hysteresis must not suppress the Z/XY/pitch safety check.
        if (maximum > float(self.p('joint_tolerance')) and
                not self.p('single_step_mode') and
                not self.p('cartesian_completion_enabled')):
            self.stable_since = None
            return
        if self.stable_since is None:
            self.stable_since = now
            return
        if (now - self.stable_since).nanoseconds / 1e9 < float(self.p('settle_time')):
            return
        actual = self.current_joints()
        if actual is None:
            self.active = False
            self.report('FAILED: joint feedback disappeared')
            return
        pose = self.fk(actual)
        pitch = float(np.sum(actual[1:4]))
        actual_step = self.last_start_z - pose[2]
        pitch_step = abs(pitch - self.last_start_pitch)
        failure = None
        if actual_step < float(self.p('minimum_actual_progress')):
            failure = f'no Z progress ({actual_step*1000:.1f}mm)'
        elif actual_step > float(self.p('max_actual_step')):
            failure = f'actual step too large ({actual_step*1000:.1f}mm)'
        elif pitch_step > float(self.p('max_pitch_step')):
            failure = f'pitch step too large ({math.degrees(pitch_step):.2f}deg)'
        elif not float(self.p('min_pitch')) <= pitch <= float(self.p('max_pitch')):
            failure = f'pitch outside range ({math.degrees(pitch):.2f}deg)'
        elif (xy_error := float(np.linalg.norm(pose[:2] - np.asarray([
                self.target.pose.position.x,
                self.target.pose.position.y])))) > float(self.p('max_xy_error')):
            failure = (
                f'actual XY error={xy_error*1000:.2f}mm exceeds '
                f'limit={float(self.p("max_xy_error"))*1000:.2f}mm')
        elif pose[2] < float(self.p('target_z_min')):
            failure = f'Z below safe band ({pose[2]:.4f}m)'
        if failure:
            self.active = False
            self.report(f'FAILED SAFE: {failure}; no next command published')
            return
        remaining = pose[2] - float(self.p('target_z_max'))
        self.report(
            f'step {self.step_index} feedback: Z={pose[2]:.4f}m, '
            f'actual drop={actual_step*1000:.1f}mm, '
            f'pitch={math.degrees(pitch):.2f}deg, remaining={max(0.0, remaining)*1000:.1f}mm')
        if float(self.p('target_z_min')) <= pose[2] <= float(self.p('target_z_max')):
            self.active = False
            self.report(
                f'COMPLETED: grasp-ready Z={pose[2]:.4f}m; '
                f'descent={self.initial_z-pose[2]:.4f}m')
            return
        if self.p('single_step_mode'):
            self.active = False
            self.report(
                f'COMPLETED STEP: Z={pose[2]:.4f}m; '
                f'actual drop={actual_step*1000:.1f}mm; no next command published')
            return
        if self.step_index >= int(self.p('max_steps')):
            self.active = False
            self.report(
                f'FAILED SAFE: max_steps reached above band at Z={pose[2]:.4f}m; '
                'no next command published')
            return
        try:
            self.publish_step(actual)
        except Exception as exception:
            self.active = False
            self.report(f'FAILED SAFE: next-step planning failed: {exception}')


def main(args=None):
    rclpy.init(args=args)
    node = MoveJClosedLoopDescent()
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
