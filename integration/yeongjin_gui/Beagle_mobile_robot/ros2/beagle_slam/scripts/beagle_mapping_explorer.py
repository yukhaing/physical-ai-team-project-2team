#!/usr/bin/env python3
"""Conservative LiDAR-guided perimeter explorer for mapping a small workcell."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class MappingExplorer(Node):
    """Map a compact rectangular workcell without a prior map.

    This is deliberately a conservative perimeter explorer, not a general
    autonomous-navigation stack. It needs a clear, static floor and an operator
    ready to use the physical emergency stop.
    """

    def __init__(self) -> None:
        super().__init__('beagle_mapping_explorer')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('forward_speed_mps', 0.025)
        self.declare_parameter('turn_speed_rps', 0.45)
        self.declare_parameter('front_stop_distance_m', 0.18)
        self.declare_parameter('turn_clearance_m', 0.14)
        self.declare_parameter('turn_angle_rad', math.pi / 2.0)
        self.declare_parameter('laps', 2)
        self.declare_parameter('max_duration_s', 180.0)
        self.declare_parameter('scan_timeout_s', 0.5)

        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10)
        self.create_subscription(
            LaserScan, str(self.get_parameter('scan_topic').value),
            self.on_scan, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value),
            self.on_odom, 10)
        self.create_service(Trigger, '~/start', self.start)
        self.create_service(Trigger, '~/stop', self.stop)
        self.create_timer(0.05, self.update)

        self.scan: LaserScan | None = None
        self.last_scan_time = 0.0
        self.yaw: float | None = None
        self.active = False
        self.state = 'IDLE'
        self.started_at = 0.0
        self.turn_start_yaw = 0.0
        self.turn_sign = 1.0
        self.turn_count = 0

    def p(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def on_scan(self, message: LaserScan) -> None:
        self.scan = message
        self.last_scan_time = time.monotonic()

    def on_odom(self, message: Odometry) -> None:
        orientation = message.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))

    def sector_clearance(self, center: float, width: float = math.radians(15)) -> float:
        if self.scan is None:
            return math.inf
        values = []
        for index, value in enumerate(self.scan.ranges):
            angle = self.scan.angle_min + index * self.scan.angle_increment
            if abs(wrap_angle(angle - center)) > width:
                continue
            distance = float(value)
            if self.scan.range_min <= distance <= self.scan.range_max:
                values.append(distance)
        return min(values, default=math.inf)

    def publish(self, *, linear: float = 0.0, angular: float = 0.0) -> None:
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.publisher.publish(message)

    def halt(self, reason: str) -> None:
        self.publish()
        self.active = False
        self.state = 'IDLE'
        self.get_logger().warning(f'Explorer stopped: {reason}')

    def start(self, request, response):
        if self.active:
            response.success = False
            response.message = 'explorer is already active'
            return response
        if self.scan is None or self.yaw is None:
            response.success = False
            response.message = 'waiting for scan or odometry'
            return response
        self.active = True
        self.state = 'FORWARD'
        self.started_at = time.monotonic()
        self.turn_count = 0
        response.success = True
        response.message = 'perimeter exploration started'
        self.get_logger().info(response.message)
        return response

    def stop(self, request, response):
        self.halt('operator stop request')
        response.success = True
        response.message = 'explorer stopped'
        return response

    def begin_turn(self) -> None:
        left = self.sector_clearance(math.pi / 2.0)
        right = self.sector_clearance(-math.pi / 2.0)
        rear = self.sector_clearance(math.pi)
        if max(left, right) < self.p('turn_clearance_m') or rear < self.p('turn_clearance_m'):
            self.halt('insufficient clearance to turn safely')
            return
        self.turn_sign = 1.0 if left >= right else -1.0
        self.turn_start_yaw = self.yaw if self.yaw is not None else 0.0
        self.state = 'TURN'
        self.get_logger().info(
            f'turning {"left" if self.turn_sign > 0 else "right"}; '
            f'front={self.sector_clearance(0):.3f}m')

    def update(self) -> None:
        if not self.active:
            return
        if time.monotonic() - self.started_at > self.p('max_duration_s'):
            self.halt('maximum mapping duration reached')
            return
        if self.scan is None or self.yaw is None or (
                time.monotonic() - self.last_scan_time > self.p('scan_timeout_s')):
            self.halt('scan or odometry timeout')
            return

        front = self.sector_clearance(0)
        if self.state == 'FORWARD':
            if front < self.p('front_stop_distance_m'):
                self.publish()
                self.begin_turn()
            else:
                self.publish(linear=self.p('forward_speed_mps'))
        elif self.state == 'TURN':
            turned = abs(wrap_angle(self.yaw - self.turn_start_yaw))
            if turned >= self.p('turn_angle_rad'):
                self.publish()
                self.turn_count += 1
                if self.turn_count >= max(1, int(self.get_parameter('laps').value)) * 4:
                    self.halt('requested perimeter laps completed')
                else:
                    self.state = 'FORWARD'
            else:
                self.publish(angular=self.turn_sign * self.p('turn_speed_rps'))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MappingExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
