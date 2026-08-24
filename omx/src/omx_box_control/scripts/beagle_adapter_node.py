#!/usr/bin/env python3
"""Simulation-first Beagle gateway; replace its transport when hardware API is known."""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BeagleAdapter(Node):
    def __init__(self):
        super().__init__('beagle_adapter')
        self.declare_parameter('command_topic', '/beagle/command')
        self.declare_parameter('status_topic', '/beagle/status')
        self.declare_parameter('simulation_arrival_seconds', 3.0)
        self.state = 'idle'
        self.job_id = None
        self.timer = None
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10)
        self.create_subscription(String, str(self.get_parameter('command_topic').value),
                                 self.on_command, 10)
        self.publish_status()

    def publish_status(self, detail=''):
        self.status_pub.publish(String(data=json.dumps({
            'state': self.state, 'job_id': self.job_id, 'detail': detail,
        })))

    def on_command(self, message):
        try:
            command = json.loads(message.data)
            name = command['command']
        except (KeyError, ValueError, TypeError) as error:
            self.get_logger().warning(f'Ignored Beagle command: {error}')
            return
        self.job_id = command.get('job_id')
        if self.timer:
            self.timer.cancel()
            self.timer = None
        if name == 'stop':
            self.state = 'stopped'
            self.publish_status('software stop requested')
            return
        if name not in ('defect_loading', 'home'):
            self.state = 'failed'
            self.publish_status(f'unsupported command: {name}')
            return
        self.state = 'returning' if name == 'home' else 'moving'
        self.publish_status(name)
        self.timer = self.create_timer(
            float(self.get_parameter('simulation_arrival_seconds').value),
            lambda: self.arrive(name))

    def arrive(self, command):
        if self.timer:
            self.timer.cancel()
            self.timer = None
        self.state = 'idle' if command == 'home' else 'arrived'
        self.publish_status(command)


def main(args=None):
    rclpy.init(args=args)
    node = BeagleAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
