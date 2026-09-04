#!/usr/bin/env python3
"""Turn a final MoveJ goal into feedback-paced joint-space waypoints."""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveJSmoothRelay(Node):
    def __init__(self):
        super().__init__('movej_smooth_relay')
        defaults = {
            'input_topic': '/omx_movej_controller/movej_raw',
            'output_topic': '/omx_movej_controller/movej',
            'joint_states_topic': '/joint_states',
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'],
            'joint_lower': [-4.71239, -2.0944, -2.0944, -1.74533, -4.71239],
            'joint_upper': [6.28319, 1.5708, 1.5708, 1.74533, 4.71239],
            'max_waypoint_delta': 0.03,
            'command_period': 0.25,
            'waypoint_duration': 0.30,
            'goal_tolerance': 0.03,
            'minimum_progress': 0.001,
            'progress_timeout': 2.0,
            'max_feedback_jump': 0.08,
            'motion_timeout': 30.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.positions = {}
        self.target = None
        self.started = None
        self.progress_reference = None
        self.progress_started = None
        self.last_feedback = None
        self.step_index = 0
        self.output = self.create_publisher(
            JointTrajectory, self.p('output_topic'), 10)
        self.status = self.create_publisher(String, '~/status', 10)
        self.create_subscription(
            JointTrajectory, self.p('input_topic'), self.on_goal, 10)
        self.create_subscription(
            JointState, self.p('joint_states_topic'), self.on_joints, 10)
        self.create_timer(float(self.p('command_period')), self.update)
        self.report('ready; smooth mode preserves the original direct MoveJ topic')

    def p(self, name):
        return self.get_parameter(name).value

    def report(self, text):
        self.get_logger().info(text)
        self.status.publish(String(data=text))

    def actual(self):
        names = list(self.p('joint_names'))
        if not all(name in self.positions for name in names):
            return None
        return np.asarray([self.positions[name] for name in names], dtype=float)

    def on_joints(self, message):
        self.positions.update(zip(message.name, message.position))

    def on_goal(self, message):
        names = list(self.p('joint_names'))
        if not message.points:
            self.report('REJECTED: empty trajectory')
            return
        actual = self.actual()
        if actual is None:
            self.report('REJECTED: joint feedback unavailable')
            return
        point = message.points[0]
        target = actual.copy()
        source_names = message.joint_names or names
        if len(point.positions) < len(source_names):
            self.report('REJECTED: positions size mismatch')
            return
        index = {name: i for i, name in enumerate(names)}
        for source_index, name in enumerate(source_names):
            if name in index:
                target[index[name]] = point.positions[source_index]
        lower = np.asarray(self.p('joint_lower'), dtype=float)
        upper = np.asarray(self.p('joint_upper'), dtype=float)
        if not np.all(np.isfinite(target)) or np.any(target < lower) or np.any(target > upper):
            self.report('REJECTED: target violates joint limits')
            return
        self.target = target
        now = self.get_clock().now()
        self.started = self.progress_started = now
        self.progress_reference = self.last_feedback = actual
        self.step_index = 0
        self.report(
            f'accepted final goal; max joint distance='
            f'{math.degrees(float(np.max(np.abs(target-actual)))):.2f}deg')

    def stop(self, reason):
        self.target = None
        self.report(reason)

    def update(self):
        if self.target is None:
            return
        actual = self.actual()
        if actual is None:
            self.stop('FAILED SAFE: joint feedback unavailable')
            return
        now = self.get_clock().now()
        if self.last_feedback is not None:
            jump = float(np.max(np.abs(actual - self.last_feedback)))
            if jump > float(self.p('max_feedback_jump')):
                self.stop(f'FAILED SAFE: feedback jump={math.degrees(jump):.2f}deg')
                return
        self.last_feedback = actual.copy()
        elapsed = (now - self.started).nanoseconds / 1e9
        if elapsed > float(self.p('motion_timeout')):
            self.stop('FAILED SAFE: smooth motion timeout')
            return
        error = self.target - actual
        maximum = float(np.max(np.abs(error)))
        if maximum <= float(self.p('goal_tolerance')):
            self.stop(f'COMPLETED: feedback error={math.degrees(maximum):.2f}deg')
            return
        progress = float(np.max(np.abs(actual - self.progress_reference)))
        if progress >= float(self.p('minimum_progress')):
            self.progress_reference = actual.copy()
            self.progress_started = now
        elif (now - self.progress_started).nanoseconds / 1e9 > float(self.p('progress_timeout')):
            self.stop('FAILED SAFE: no measured joint progress')
            return
        scale = min(1.0, float(self.p('max_waypoint_delta')) / maximum)
        waypoint = actual + scale * error
        message = JointTrajectory()
        message.joint_names = list(self.p('joint_names'))
        point = JointTrajectoryPoint()
        point.positions = waypoint.tolist()
        duration = float(self.p('waypoint_duration'))
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)
        message.points = [point]
        self.output.publish(message)
        self.step_index += 1
        self.report(
            f'waypoint {self.step_index}: remaining={math.degrees(maximum):.2f}deg, '
            f'command step={math.degrees(float(np.max(np.abs(waypoint-actual)))):.2f}deg')


def main(args=None):
    rclpy.init(args=args)
    node = MoveJSmoothRelay()
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
