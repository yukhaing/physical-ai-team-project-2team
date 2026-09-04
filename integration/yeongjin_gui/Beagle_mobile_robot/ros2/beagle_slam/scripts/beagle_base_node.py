#!/usr/bin/env python3
"""Expose the Robomation Beagle as a small ROS 2 differential-drive base."""

from __future__ import annotations

import math
from pathlib import Path
import site
import time

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_quaternion(yaw: float) -> Quaternion:
    quaternion = Quaternion()
    quaternion.z = math.sin(yaw / 2.0)
    quaternion.w = math.cos(yaw / 2.0)
    return quaternion


def encoder_delta(current: int, previous: int) -> int:
    """Return a signed delta while tolerating a 32-bit encoder rollover."""
    delta = int(current) - int(previous)
    if delta > 0x7FFFFFFF:
        delta -= 0x100000000
    elif delta < -0x80000000:
        delta += 0x100000000
    return delta


class BeagleBase(Node):
    def __init__(self) -> None:
        super().__init__('beagle_base')
        self.declare_parameter(
            'roboid_site_packages',
            '/root/omx_box_project_ws/integration/yeongjin_gui/'
            'Beagle_mobile_robot/.venv/lib/python3.12/site-packages')
        self.declare_parameter('port_name', '/dev/ttyACM0')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('laser_x_m', 0.0)
        self.declare_parameter('laser_y_m', 0.0)
        self.declare_parameter('laser_z_m', 0.10)
        self.declare_parameter('wheel_base_m', 0.0956)
        self.declare_parameter('wheel_radius_m', 0.033)
        self.declare_parameter('max_rpm', 93.75)
        # The API documentation does not state the encoder's physical unit.
        # The current project mock treats one unit as one millimetre. Verify
        # this value on the real robot before relying on the generated map.
        self.declare_parameter('meters_per_encoder_unit', 0.0001073)
        self.declare_parameter('encoder_left_sign', 1.0)
        self.declare_parameter('encoder_right_sign', 1.0)
        self.declare_parameter('maximum_wheel_percent', 18.0)
        self.declare_parameter('forward_stop_distance_m', 0.14)
        self.declare_parameter('reverse_stop_distance_m', 0.14)
        self.declare_parameter('command_timeout_s', 0.4)
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('scan_rate_hz', 8.0)
        self.declare_parameter('range_min_m', 0.08)
        self.declare_parameter('range_max_m', 5.0)

        roboid_path = Path(str(self.get_parameter('roboid_site_packages').value))
        if not roboid_path.is_dir():
            raise RuntimeError(f'roboid site-packages not found: {roboid_path}')
        site.addsitedir(str(roboid_path))
        try:
            from roboid import Beagle  # type: ignore
        except ImportError as error:
            raise RuntimeError(f'failed to import roboid from {roboid_path}') from error

        port_name = str(self.get_parameter('port_name').value).strip()
        self.get_logger().info(f'Connecting to Beagle on {port_name or "auto"}')
        self.robot = Beagle(0, port_name or None)
        self.robot.stop()
        self.robot.reset_encoder()
        self.robot.start_lidar()
        lidar_deadline = time.monotonic() + 8.0
        while time.monotonic() < lidar_deadline:
            if self.robot.is_lidar_ready():
                break
            time.sleep(0.05)
        else:
            self.robot.stop()
            self.robot.dispose()
            raise RuntimeError(
                f'Beagle/LiDAR unavailable on {port_name}; LiDAR was not ready after 8 seconds')

        self.scan_pub = self.create_publisher(
            LaserScan, 'scan', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 20)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.tf_pub = TransformBroadcaster(self)
        self.static_tf_pub = StaticTransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.previous_left = int(self.robot.left_encoder())
        self.previous_right = int(self.robot.right_encoder())
        self.previous_update = time.monotonic()
        self.last_command = time.monotonic()
        self.command_active = False
        self.last_scan = 0.0
        self.publish_laser_transform()

        rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate, self.update)
        self.get_logger().info(
            'Beagle base ready: publishing /scan, /odom and odom->base_link; '
            'subscribing to /cmd_vel')

    def value(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def publish_laser_transform(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = str(self.get_parameter('base_frame').value)
        transform.child_frame_id = str(self.get_parameter('laser_frame').value)
        transform.transform.translation.x = self.value('laser_x_m')
        transform.transform.translation.y = self.value('laser_y_m')
        transform.transform.translation.z = self.value('laser_z_m')
        transform.transform.rotation.w = 1.0
        self.static_tf_pub.sendTransform(transform)

    def on_cmd_vel(self, message: Twist) -> None:
        linear = float(message.linear.x)
        if linear > 0.0 and self._clearance_m(0) < self.value("forward_stop_distance_m"):
            self.robot.stop()
            self.command_active = False
            self.get_logger().warning("Forward command blocked: obstacle is too close")
            return
        if linear < 0.0 and self._clearance_m(180) < self.value("reverse_stop_distance_m"):
            self.robot.stop()
            self.command_active = False
            self.get_logger().warning("Reverse command blocked: obstacle is too close")
            return
        wheel_base = self.value("wheel_base_m")
        left_mps = linear - float(message.angular.z) * wheel_base / 2.0
        right_mps = linear + float(message.angular.z) * wheel_base / 2.0
        max_mps = (
            2.0 * math.pi * self.value("wheel_radius_m") *
            self.value("max_rpm") / 60.0)
        if max_mps <= 0.0:
            self.robot.stop()
            return
        limit = abs(self.value("maximum_wheel_percent"))
        left_percent = clamp(100.0 * left_mps / max_mps, -limit, limit)
        right_percent = clamp(100.0 * right_mps / max_mps, -limit, limit)
        self.robot.wheels(left_percent, right_percent)
        self.last_command = time.monotonic()
        self.command_active = abs(left_percent) > 0.01 or abs(right_percent) > 0.01

    def _clearance_m(self, center_deg: int) -> float:
        """Return the nearest valid distance in a 21-degree LiDAR sector."""
        raw = list(self.robot.lidar())
        values = []
        for offset in range(-10, 11):
            try:
                distance = float(raw[(center_deg + offset) % 360]) / 1000.0
            except (IndexError, TypeError, ValueError, OverflowError):
                continue
            if 0.08 <= distance <= 5.0:
                values.append(distance)
        return min(values, default=math.inf)

    def update(self) -> None:
        now_monotonic = time.monotonic()
        if (self.command_active and
                now_monotonic - self.last_command > self.value('command_timeout_s')):
            self.robot.stop()
            self.command_active = False

        current_left = int(self.robot.left_encoder())
        current_right = int(self.robot.right_encoder())
        scale = self.value('meters_per_encoder_unit')
        delta_left = (
            encoder_delta(current_left, self.previous_left) * scale *
            self.value('encoder_left_sign'))
        delta_right = (
            encoder_delta(current_right, self.previous_right) * scale *
            self.value('encoder_right_sign'))
        self.previous_left = current_left
        self.previous_right = current_right

        dt = max(1e-3, now_monotonic - self.previous_update)
        self.previous_update = now_monotonic
        distance = (delta_left + delta_right) / 2.0
        delta_yaw = (delta_right - delta_left) / self.value('wheel_base_m')
        midpoint_yaw = self.yaw + delta_yaw / 2.0
        self.x += distance * math.cos(midpoint_yaw)
        self.y += distance * math.sin(midpoint_yaw)
        self.yaw = wrap_angle(self.yaw + delta_yaw)

        stamp = self.get_clock().now().to_msg()
        linear = distance / dt
        angular = delta_yaw / dt
        self.publish_odometry(stamp, linear, angular)

        scan_rate = max(1.0, self.value('scan_rate_hz'))
        if now_monotonic - self.last_scan >= 1.0 / scan_rate:
            self.last_scan = now_monotonic
            self.publish_scan(stamp)

    def publish_odometry(self, stamp, linear: float, angular: float) -> None:
        quaternion = yaw_quaternion(self.yaw)
        odom_frame = str(self.get_parameter('odom_frame').value)
        base_frame = str(self.get_parameter('base_frame').value)

        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = odom_frame
        message.child_frame_id = base_frame
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.orientation = quaternion
        message.pose.covariance[0] = 0.02
        message.pose.covariance[7] = 0.02
        message.pose.covariance[35] = 0.05
        message.twist.twist.linear.x = linear
        message.twist.twist.angular.z = angular
        message.twist.covariance[0] = 0.04
        message.twist.covariance[35] = 0.08
        self.odom_pub.publish(message)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = odom_frame
        transform.child_frame_id = base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = quaternion
        self.tf_pub.sendTransform(transform)

    def publish_scan(self, stamp) -> None:
        raw = list(self.robot.lidar())
        if len(raw) != 360:
            self.get_logger().warning(
                f'Expected 360 LiDAR samples, received {len(raw)}',
                throttle_duration_sec=2.0)
            return
        # Roboid index 0 is front, 90 is left and angles increase CCW.
        # Reorder [180..359, 0..179] so LaserScan spans [-pi, pi).
        ordered = raw[180:] + raw[:180]
        range_min = self.value('range_min_m')
        range_max = self.value('range_max_m')
        ranges = []
        for value in ordered:
            try:
                raw_value = int(value)
                distance = raw_value / 1000.0
            except (TypeError, ValueError, OverflowError):
                raw_value = 65535
                distance = math.inf
            if raw_value == 65535 or not range_min <= distance <= range_max:
                distance = math.nan
            ranges.append(distance)

        increment = 2.0 * math.pi / len(ranges)
        message = LaserScan()
        message.header.stamp = stamp
        message.header.frame_id = str(self.get_parameter('laser_frame').value)
        message.angle_min = -math.pi
        message.angle_max = math.pi - increment
        message.angle_increment = increment
        message.range_min = range_min
        message.range_max = range_max
        message.ranges = ranges
        self.scan_pub.publish(message)

    def close(self) -> None:
        try:
            self.robot.stop()
            self.robot.stop_lidar()
            self.robot.dispose()
        except Exception as error:  # noqa: BLE001 - hardware cleanup boundary
            self.get_logger().warning(f'Beagle cleanup failed: {error}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = BeagleBase()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
