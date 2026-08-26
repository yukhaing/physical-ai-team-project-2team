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
        self.declare_parameter('selection_topic', '/console/selected_box')
        self.declare_parameter('pixel_selection_topic', '/console/select_pixel')
        self.declare_parameter('status_topic', '/console/status')
        self.declare_parameter('model_path', '/root/omx_box_project_ws/models/best.pt')
        self.declare_parameter('confidence_threshold', 0.50)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('auto_select_defect', False)
        self.declare_parameter('auto_select_min_interval_sec', 8.0)
        try:
            from ultralytics import YOLO
            self.model = YOLO(str(self.get_parameter('model_path').value))
        except Exception as error:
            raise RuntimeError(f'Unable to load YOLO model: {error}') from error
        self.bridge = CvBridge()
        self.last_auto_select_time = None
        self.console_status = ''
        self.image_pub = self.create_publisher(
            Image, str(self.get_parameter('annotated_image_topic').value), 2)
        self.detections_pub = self.create_publisher(
            String, str(self.get_parameter('detections_topic').value), 10)
        self.selection_pub = self.create_publisher(
            String, str(self.get_parameter('selection_topic').value), 10)
        self.pixel_pub = self.create_publisher(
            String, str(self.get_parameter('pixel_selection_topic').value), 10)
        self.create_subscription(Image, str(self.get_parameter('image_topic').value),
                                 self.on_image, 2)
        self.create_subscription(
            String, str(self.get_parameter('status_topic').value), self.on_status, 10)
        self.get_logger().info('YOLO detector ready')

    @staticmethod
    def to_image_msg(frame, header):
        message = Image()
        message.header = header
        message.height = int(frame.shape[0])
        message.width = int(frame.shape[1])
        message.encoding = 'bgr8'
        message.is_bigendian = False
        message.step = int(frame.shape[1] * frame.shape[2])
        message.data = frame.tobytes()
        return message

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
            self.image_pub.publish(self.to_image_msg(frame, message.header))
            self.detections_pub.publish(String(data=json.dumps({'detections': detections})))
            self.auto_select_defect(detections)
        except Exception as error:
            self.get_logger().error(f'YOLO inference failed: {error}')

    def on_status(self, message):
        self.console_status = message.data.strip()

    def auto_select_defect(self, detections):
        if not bool(self.get_parameter('auto_select_defect').value):
            return
        if not (
                self.console_status.startswith('READY:') or
                self.console_status.startswith('BEAGLE_HOME:')):
            return
        defects = [d for d in detections if d.get('class') == 'defect']
        if not defects:
            return
        now = self.get_clock().now()
        if self.last_auto_select_time is not None:
            elapsed = (now - self.last_auto_select_time).nanoseconds / 1e9
            if elapsed < float(self.get_parameter('auto_select_min_interval_sec').value):
                return
        detection = max(defects, key=lambda item: float(item.get('confidence', 0.0)))
        selection = dict(
            detection,
            x=int(round(float(detection['center_x']))),
            y=int(round(float(detection['center_y']))),
            source='auto_detect')
        payload = String(data=json.dumps(selection))
        self.pixel_pub.publish(payload)
        self.selection_pub.publish(payload)
        self.last_auto_select_time = now
        self.get_logger().info(
            f"Auto-selected defect at pixel=({selection['x']}, {selection['y']}) "
            f"conf={float(detection['confidence']):.2f}")


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
