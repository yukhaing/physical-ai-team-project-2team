#!/usr/bin/env python3

import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from robotis_interfaces.msg import MoveL


class BoxTargetPoseBridge(Node):
    """Validate link0 targets and convert them to OMX MoveL commands."""

    def __init__(self):
        super().__init__('box_target_pose_bridge')
        self.declare_parameter('input_topic', '/box_target_pose')
        self.declare_parameter('output_topic', '/omx_movel_controller/movel')
        self.declare_parameter('base_frame', 'link0')
        self.declare_parameter('move_duration', 1.0)
        self.declare_parameter('min_x', 0.08)
        self.declare_parameter('max_x', 0.32)
        self.declare_parameter('max_abs_y', 0.25)
        self.declare_parameter('min_z', 0.01)
        self.declare_parameter('max_z', 0.32)
        self.declare_parameter('min_reach', 0.10)
        self.declare_parameter('max_reach', 0.42)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.publisher = self.create_publisher(MoveL, output_topic, 10)
        self.subscription = self.create_subscription(
            PoseStamped, input_topic, self.target_callback, 10)
        self.get_logger().info(f'OMX target bridge: {input_topic} -> {output_topic}')

    def target_callback(self, target):
        expected_frame = self.get_parameter('base_frame').value
        if target.header.frame_id != expected_frame:
            self.get_logger().error(
                f"Rejected frame '{target.header.frame_id}'; expected '{expected_frame}'")
            return

        p = target.pose.position
        if not self.position_is_safe(p.x, p.y, p.z):
            self.get_logger().error(
                f'Rejected unsafe target: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}')
            return

        command = MoveL()
        command.pose.header.stamp = self.get_clock().now().to_msg()
        command.pose.header.frame_id = expected_frame
        command.pose.pose.position = p

        # Keep the tool horizontal and rotate it radially toward the target.
        yaw = math.atan2(p.y, p.x + 0.01125)
        command.pose.pose.orientation.z = math.sin(yaw * 0.5)
        command.pose.pose.orientation.w = math.cos(yaw * 0.5)

        duration = max(0.02, float(self.get_parameter('move_duration').value))
        command.time_from_start.sec = int(duration)
        command.time_from_start.nanosec = int((duration % 1.0) * 1.0e9)
        self.publisher.publish(command)
        self.get_logger().info(
            f'Published OMX target: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}')

    def position_is_safe(self, x, y, z):
        if not all(math.isfinite(value) for value in (x, y, z)):
            return False
        reach = math.sqrt(x * x + y * y + z * z)
        return (
            float(self.get_parameter('min_x').value) <= x
            <= float(self.get_parameter('max_x').value)
            and abs(y) <= float(self.get_parameter('max_abs_y').value)
            and float(self.get_parameter('min_z').value) <= z
            <= float(self.get_parameter('max_z').value)
            and float(self.get_parameter('min_reach').value) <= reach
            <= float(self.get_parameter('max_reach').value)
        )


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
