#!/usr/bin/env python3
# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0

import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from robotis_interfaces.msg import MoveL


class BoxTargetPoseBridge(Node):
    """Convert PoseStamped targets into OMX MoveL commands."""

    def __init__(self):
        super().__init__('box_target_pose_bridge')
        self.declare_parameter('input_topic', '/box_target_pose')
        self.declare_parameter('output_topic', '/omx_movel_controller/movel')
        self.declare_parameter('base_frame', 'link0')
        self.declare_parameter('move_duration', 1.0)
        self.declare_parameter('lock_orientation', True)
        self.declare_parameter('horizontal_radial_orientation', True)
        self.declare_parameter('fixed_qx', 0.0)
        self.declare_parameter('fixed_qy', 0.0)
        self.declare_parameter('fixed_qz', 0.0)
        self.declare_parameter('fixed_qw', 1.0)
        self.declare_parameter('min_x', 0.08)
        self.declare_parameter('max_x', 0.32)
        self.declare_parameter('max_abs_y', 0.25)
        self.declare_parameter('min_z', 0.01)
        self.declare_parameter('max_z', 0.32)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.publisher = self.create_publisher(MoveL, output_topic, 10)
        self.subscription = self.create_subscription(
            PoseStamped, input_topic, self.target_callback, 10)
        self.get_logger().info(f'Pose bridge ready: {input_topic} -> {output_topic}')
        self.get_logger().info(
            f"Expected frame: {self.get_parameter('base_frame').value}; "
            f"orientation lock: {self.get_parameter('lock_orientation').value}")

    def target_callback(self, target):
        expected_frame = self.get_parameter('base_frame').value
        if target.header.frame_id != expected_frame:
            self.get_logger().error(
                f"Rejected target frame '{target.header.frame_id}'; "
                f"expected '{expected_frame}'")
            return

        p = target.pose.position
        if not self.position_is_safe(p.x, p.y, p.z):
            self.get_logger().error(
                f'Rejected target outside configured workspace: '
                f'x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}')
            return

        command = MoveL()
        command.pose.header.stamp = self.get_clock().now().to_msg()
        command.pose.header.frame_id = expected_frame
        command.pose.pose.position = p

        if (self.get_parameter('lock_orientation').value
                and self.get_parameter('horizontal_radial_orientation').value):
            yaw = math.atan2(p.y, p.x + 0.01125)
            command.pose.pose.orientation.z = math.sin(yaw * 0.5)
            command.pose.pose.orientation.w = math.cos(yaw * 0.5)
        elif self.get_parameter('lock_orientation').value:
            command.pose.pose.orientation.x = float(self.get_parameter('fixed_qx').value)
            command.pose.pose.orientation.y = float(self.get_parameter('fixed_qy').value)
            command.pose.pose.orientation.z = float(self.get_parameter('fixed_qz').value)
            command.pose.pose.orientation.w = float(self.get_parameter('fixed_qw').value)
        else:
            q = target.pose.orientation
            norm = math.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2)
            if norm < 1.0e-9:
                self.get_logger().error('Rejected target with invalid orientation')
                return
            command.pose.pose.orientation.x = q.x / norm
            command.pose.pose.orientation.y = q.y / norm
            command.pose.pose.orientation.z = q.z / norm
            command.pose.pose.orientation.w = q.w / norm

        duration = max(0.02, float(self.get_parameter('move_duration').value))
        command.time_from_start.sec = int(duration)
        command.time_from_start.nanosec = int((duration % 1.0) * 1.0e9)
        self.publisher.publish(command)
        self.get_logger().info(
            f'Published OMX target: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}')

    def position_is_safe(self, x, y, z):
        if not all(math.isfinite(value) for value in (x, y, z)):
            return False
        return (
            float(self.get_parameter('min_x').value) <= x
            <= float(self.get_parameter('max_x').value)
            and abs(y) <= float(self.get_parameter('max_abs_y').value)
            and float(self.get_parameter('min_z').value) <= z
            <= float(self.get_parameter('max_z').value))


def main(args=None):
    rclpy.init(args=args)
    node = BoxTargetPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
