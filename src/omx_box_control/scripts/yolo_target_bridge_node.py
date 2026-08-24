#!/usr/bin/env python3
"""Validate a stable YOLO target before handing it to the pick coordinator."""

from collections import deque
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String


class YoloTargetBridge(Node):
    def __init__(self):
        super().__init__('yolo_target_bridge')
        defaults = {
            'input_topic': '/yolo/selected_box',
            'output_topic': '/camera_box_target',
            'coordinator_status_topic': '/pick_coordinator/status',
            'base_frame': 'link0',
            'target_z': 0.10,
            'defect_only': True,
            'minimum_confidence': 0.35,
            'sample_count': 4,
            'maximum_xy_spread': 0.012,
            'min_x': 0.08,
            'max_x': 0.32,
            'max_abs_y': 0.25,
            'max_radius': 0.31,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        count = max(2, int(self.p('sample_count')))
        self.samples = deque(maxlen=count)
        self.waiting_for_target = False
        self.published_for_cycle = False
        self.target_pub = self.create_publisher(
            PoseStamped, self.p('output_topic'), 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.create_subscription(
            Float64MultiArray, self.p('input_topic'), self.on_detection, 10)
        self.create_subscription(
            String, self.p('coordinator_status_topic'), self.on_coordinator_status, 10)
        self.report('ready; waiting for coordinator WAIT_PICK_TARGET')

    def p(self, name):
        return self.get_parameter(name).value

    def report(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def on_coordinator_status(self, message):
        state = message.data.split(':', 1)[0]
        waiting = state == 'WAIT_PICK_TARGET'
        if waiting and not self.waiting_for_target:
            self.samples.clear()
            self.published_for_cycle = False
            self.report('coordinator ready; collecting a stable YOLO target')
        self.waiting_for_target = waiting

    def on_detection(self, message):
        if not self.waiting_for_target or self.published_for_cycle:
            return
        if len(message.data) != 5:
            self.report(f'rejected YOLO message length={len(message.data)}; expected 5')
            return

        is_defect, confidence, x, y, joint5 = map(float, message.data)
        if bool(self.p('defect_only')) and is_defect < 0.5:
            return
        if confidence < float(self.p('minimum_confidence')):
            return
        if not all(math.isfinite(value) for value in (confidence, x, y)):
            self.report('rejected non-finite YOLO target')
            return
        if not (float(self.p('min_x')) <= x <= float(self.p('max_x'))):
            self.report(f'rejected X outside workspace: {x:.4f}m')
            return
        if abs(y) > float(self.p('max_abs_y')):
            self.report(f'rejected Y outside workspace: {y:.4f}m')
            return
        if math.hypot(x, y) > float(self.p('max_radius')):
            self.report(f'rejected radius outside workspace: {math.hypot(x, y):.4f}m')
            return

        self.samples.append((x, y, confidence, joint5))
        if len(self.samples) < self.samples.maxlen:
            return
        values = np.asarray(self.samples, dtype=float)
        spread = np.ptp(values[:, :2], axis=0)
        limit = float(self.p('maximum_xy_spread'))
        if np.any(spread > limit):
            self.report(
                f'unstable YOLO target; spread x={spread[0]:.4f}m, y={spread[1]:.4f}m')
            return

        x_median, y_median = np.median(values[:, :2], axis=0)
        confidence_median = float(np.median(values[:, 2]))
        finite_joint5 = values[np.isfinite(values[:, 3]), 3]
        joint5_median = (
            float(np.median(finite_joint5)) if finite_joint5.size else float('nan'))

        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = str(self.p('base_frame'))
        target.pose.position.x = float(x_median)
        target.pose.position.y = float(y_median)
        target.pose.position.z = float(self.p('target_z'))
        target.pose.orientation.w = 1.0
        self.target_pub.publish(target)
        self.published_for_cycle = True
        q5_text = f'{joint5_median:.4f}rad' if math.isfinite(joint5_median) else 'unavailable'
        self.report(
            f'published stable defect target x={x_median:.4f}m, y={y_median:.4f}m, '
            f'confidence={confidence_median:.3f}, detected joint5={q5_text}; '
            'joint5 is not commanded by this bridge')


def main(args=None):
    rclpy.init(args=args)
    node = YoloTargetBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
