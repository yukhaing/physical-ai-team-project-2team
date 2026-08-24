#!/usr/bin/env python3
"""Publish YOLO annotations and compact JSON detections for the operator console."""

import json

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection')
        self.declare_parameter('image_topic', '/camera1/image_raw')
        self.declare_parameter('annotated_image_topic', '/console/annotated_image')
        self.declare_parameter('detections_topic', '/console/detections')
        self.declare_parameter('model_path', '/root/omx_box_project_ws/models/best.pt')
        self.declare_parameter('confidence_threshold', 0.50)
        self.declare_parameter('device', 'cpu')
        try:
            from ultralytics import YOLO
            self.model = YOLO(str(self.get_parameter('model_path').value))
        except Exception as error:
            raise RuntimeError(f'Unable to load YOLO model: {error}') from error
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(
            Image, str(self.get_parameter('annotated_image_topic').value), 2)
        self.detections_pub = self.create_publisher(
            String, str(self.get_parameter('detections_topic').value), 10)
        self.create_subscription(Image, str(self.get_parameter('image_topic').value),
                                 self.on_image, 2)
        self.get_logger().info('YOLO detector ready')

    def on_image(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            result = self.model.predict(
                frame, conf=float(self.get_parameter('confidence_threshold').value),
                device=str(self.get_parameter('device').value), verbose=False)[0]
            detections = []
            names = result.names
            for box in result.boxes:
                x1, y1, x2, y2 = [float(value) for value in box.xyxy[0].tolist()]
                class_id = int(box.cls[0].item())
                label = str(names[class_id])
                confidence = float(box.conf[0].item())
                detections.append({
                    'class': label, 'confidence': confidence,
                    'x': x1, 'y': y1, 'width': x2 - x1, 'height': y2 - y1,
                    'center_x': (x1 + x2) / 2.0, 'center_y': (y1 + y2) / 2.0,
                })
                color = (55, 200, 55) if label == 'normal' else (40, 60, 235)
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                cv2.putText(frame, f'{label} {confidence:.2f}', (int(x1), max(18, int(y1) - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
            self.detections_pub.publish(String(data=json.dumps({'detections': detections})))
        except Exception as error:
            self.get_logger().error(f'YOLO inference failed: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
