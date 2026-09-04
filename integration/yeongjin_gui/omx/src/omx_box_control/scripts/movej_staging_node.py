#!/usr/bin/env python3
"""Move OMX-F to a verified joint-space staging configuration."""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveJStaging(Node):
    def __init__(self):
        super().__init__('movej_staging')
        defaults = {
            'movej_topic': '/omx_movej_controller/movej',
            'joint_states_topic': '/joint_states',
            'controller_error_topic': '/omx_movej_controller/controller_error',
            'dry_run': True,
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'],
            'staging_positions': [0.0, -0.467, 0.376, 1.291, 0.0],
            'preserve_joint_names': [''],
            'move_duration': 6.0,
            'minimum_completion_time': 6.0,
            'joint_tolerance': 0.03,
            'settle_time': 0.30,
            'timeout': 15.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.positions = {}
        self.goal = None
        self.started = self.stable_since = None
        self.active = False
        self.publisher = self.create_publisher(
            JointTrajectory, self.get_parameter('movej_topic').value, 10)
        self.status_publisher = self.create_publisher(String, '~/status', 10)
        self.create_subscription(
            JointState, self.get_parameter('joint_states_topic').value,
            self.on_joint_state, 10)
        self.create_subscription(
            String, self.get_parameter('controller_error_topic').value,
            self.on_controller_error, 10)
        self.create_service(Trigger, '~/confirm', self.on_confirm)
        self.create_service(Trigger, '~/cancel', self.on_cancel)
        self.create_timer(0.05, self.update)
        self.report('ready; call ~/confirm with the MoveJ controller active')

    def report(self, text):
        self.status_publisher.publish(String(data=text))
        self.get_logger().info(text)

    def on_joint_state(self, message):
        self.positions.update(zip(message.name, message.position))

    def on_controller_error(self, message):
        if self.active:
            self.report(f'WARNING: {message.data}; continuing until joint timeout')

    def on_confirm(self, _request, response):
        if self.active:
            response.success, response.message = False, 'staging move already active'
            return response
        names = list(self.get_parameter('joint_names').value)
        goal = [float(value) for value in self.get_parameter('staging_positions').value]
        if len(names) != len(goal) or not names:
            response.success, response.message = False, 'joint_names and staging_positions size mismatch'
            return response
        if not all(name in self.positions for name in names):
            response.success, response.message = False, 'joint state feedback unavailable'
            return response
        preserve = {name for name in self.get_parameter('preserve_joint_names').value if name}
        unknown = preserve.difference(names)
        if unknown:
            response.success, response.message = False, (
                f'preserve_joint_names contains unknown joints: {sorted(unknown)}')
            return response
        for index, name in enumerate(names):
            if name in preserve:
                goal[index] = float(self.positions[name])
        if not all(math.isfinite(value) for value in goal):
            response.success, response.message = False, 'staging positions must be finite'
            return response
        if self.get_parameter('dry_run').value:
            response.success, response.message = False, 'dry_run=true; command disabled'
            self.report(response.message)
            return response
        message = JointTrajectory()
        message.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = goal
        duration = max(0.02, float(self.get_parameter('move_duration').value))
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1.0e9)
        message.points = [point]
        self.publisher.publish(message)
        self.goal = dict(zip(names, goal))
        self.started, self.stable_since = self.get_clock().now(), None
        self.active = True
        preserved = f'; preserving {sorted(preserve)}' if preserve else ''
        self.report(f'moving to MoveJ staging configuration{preserved}')
        response.success, response.message = True, 'MoveJ staging started'
        return response

    def on_cancel(self, _request, response):
        was_active = self.active
        self.active = False
        names = list(self.goal) if self.goal else list(self.get_parameter('joint_names').value)
        if names and all(name in self.positions for name in names):
            message = JointTrajectory()
            message.joint_names = names
            point = JointTrajectoryPoint()
            point.positions = [self.positions[name] for name in names]
            point.time_from_start.nanosec = 100000000
            message.points = [point]
            self.publisher.publish(message)
        self.report('cancelled; holding current joints')
        response.success, response.message = was_active, 'cancelled' if was_active else 'already idle'
        return response

    def update(self):
        if not self.active:
            return
        now = self.get_clock().now()
        elapsed = (now - self.started).nanoseconds / 1e9
        errors = [abs(self.positions.get(name, math.inf) - goal) for name, goal in self.goal.items()]
        maximum = max(errors)
        if (now - self.started).nanoseconds / 1e9 > float(self.get_parameter('timeout').value):
            self.active = False
            self.report(f'FAILED: timeout; maximum joint error={math.degrees(maximum):.2f}deg')
            return
        if maximum > float(self.get_parameter('joint_tolerance').value):
            self.stable_since = None
            return
        if elapsed < float(self.get_parameter('minimum_completion_time').value):
            self.stable_since = None
            return
        if self.stable_since is None:
            self.stable_since = now
            return
        if (now - self.stable_since).nanoseconds / 1e9 >= float(self.get_parameter('settle_time').value):
            self.active = False
            self.report(f'COMPLETED: maximum joint error={math.degrees(maximum):.2f}deg')


def main(args=None):
    rclpy.init(args=args)
    node = MoveJStaging()
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
