#!/usr/bin/env python3
"""Publish YOLO annotations and compact JSON detections for the operator console."""

from collections import deque
import json
import math
import os

import cv2
from cv_bridge import CvBridge
import numpy as np
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
        self.declare_parameter('angle_calibration_file', '')
        self.declare_parameter('angle_min_contour_area', 1000.0)
        self.declare_parameter('joint5_offset_deg', 2.00255)
        self.declare_parameter('joint5_sample_count', 4)
        self.declare_parameter('joint5_period', math.pi / 2.0)
        self.declare_parameter('maximum_joint5_spread', 0.18)
        self.declare_parameter('minimum_joint5', -0.80)
        self.declare_parameter('maximum_joint5', 0.80)
        try:
            from ultralytics import YOLO
            self.model = YOLO(str(self.get_parameter('model_path').value))
        except Exception as error:
            raise RuntimeError(f'Unable to load YOLO model: {error}') from error
        self.bridge = CvBridge()
        self.homography, self.calibration_pixels, self.calibration_residuals = (
            self.load_angle_calibration())
        self.joint5_samples = deque(maxlen=max(
            1, int(self.get_parameter('joint5_sample_count').value)))
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

    def load_angle_calibration(self):
        path = str(self.get_parameter('angle_calibration_file').value)
        if not path or not os.path.isfile(path):
            raise RuntimeError(f'Angle calibration file not found: {path}')
        storage = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        matrix = storage.getNode('homography').mat() if storage.isOpened() else None
        image = storage.getNode('image_points').mat() if storage.isOpened() else None
        reference = storage.getNode('reference_points_link0').mat() if storage.isOpened() else None
        storage.release()
        if matrix is None or matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
            raise RuntimeError(f'Invalid angle homography: {path}')
        if image is None or reference is None:
            raise RuntimeError(f'Angle calibration points are missing: {path}')
        pixels = image.reshape((-1, 2)).astype(float)
        reference = reference.reshape((-1, 2)).astype(float)
        projected = cv2.perspectiveTransform(
            pixels.reshape((-1, 1, 2)), matrix).reshape((-1, 2))
        self.get_logger().info(f'Loaded joint5 angle calibration from {path}')
        return matrix, pixels, reference - projected

    def world(self, point):
        pixel = np.asarray(point, dtype=float)
        transformed = self.homography @ np.array([pixel[0], pixel[1], 1.0])
        raw = transformed[:2] / transformed[2]
        distance = np.linalg.norm(self.calibration_pixels - pixel, axis=1)
        nearest = int(np.argmin(distance))
        if distance[nearest] < 1.0:
            return raw + self.calibration_residuals[nearest]
        weights = 1.0 / (distance * distance + 1e-6)
        correction = (
            weights[:, None] * self.calibration_residuals).sum(axis=0) / weights.sum()
        return raw + correction

    @staticmethod
    def normalize_90_degrees(angle):
        return (angle + 45.0) % 90.0 - 45.0

    def estimate_joint5(self, frame, bounds, center):
        x1, y1, x2, y2 = [int(round(value)) for value in bounds]
        xa, ya = max(0, x1 - 8), max(0, y1 - 8)
        xb, yb = min(frame.shape[1], x2 + 8), min(frame.shape[0], y2 + 8)
        if xb <= xa or yb <= ya:
            return None
        hsv = cv2.cvtColor(frame[ya:yb, xa:xb], cv2.COLOR_BGR2HSV)
        mask = ((hsv[:, :, 1] > 18) & (hsv[:, :, 1] < 150) &
                (hsv[:, :, 2] < 245) & (hsv[:, :, 0] > 4) &
                (hsv[:, :, 0] < 35)).astype(np.uint8) * 255
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [contour for contour in contours if cv2.contourArea(contour) >
                    float(self.get_parameter('angle_min_contour_area').value)]
        if not contours:
            return None
        rotated_box = cv2.boxPoints(cv2.minAreaRect(max(contours, key=cv2.contourArea)))
        rotated_box[:, 0] += xa
        rotated_box[:, 1] += ya
        edges = [
            (np.linalg.norm(rotated_box[(index + 1) % 4] - rotated_box[index]),
             rotated_box[index], rotated_box[(index + 1) % 4])
            for index in range(4)
        ]
        _length, start, end = max(edges, key=lambda item: item[0])
        world_start, world_end = self.world(start), self.world(end)
        axis = self.normalize_90_degrees(math.degrees(math.atan2(
            world_end[1] - world_start[1], world_end[0] - world_start[0])))
        robot_x, robot_y = self.world(center)
        joint1 = math.degrees(math.atan2(robot_y, robot_x + 0.01125))
        joint5_deg = self.normalize_90_degrees(
            joint1 + 90.0 - axis +
            float(self.get_parameter('joint5_offset_deg').value))
        joint5 = math.radians(joint5_deg)
        if not (float(self.get_parameter('minimum_joint5').value) <= joint5 <=
                float(self.get_parameter('maximum_joint5').value)):
            return None
        return rotated_box, axis, joint5, float(robot_x), float(robot_y)

    def stabilize_best_defect_angle(self, detections):
        candidates = [d for d in detections if d.get('class') == 'defect' and
                      math.isfinite(float(d.get('joint5_raw_rad', float('nan'))))]
        if not candidates:
            self.joint5_samples.clear()
            return
        selected = max(candidates, key=lambda item: float(item['confidence']))
        self.joint5_samples.append(float(selected['joint5_raw_rad']))
        if len(self.joint5_samples) < self.joint5_samples.maxlen:
            return
        values = np.asarray(self.joint5_samples, dtype=float)
        period = float(self.get_parameter('joint5_period').value)
        phase = values * (2.0 * math.pi / period)
        median = math.atan2(float(np.mean(np.sin(phase))),
                            float(np.mean(np.cos(phase)))) * period / (2.0 * math.pi)
        error = (values - median + period / 2.0) % period - period / 2.0
        spread = float(np.ptp(error))
        if spread > float(self.get_parameter('maximum_joint5_spread').value):
            return
        selected['joint5_rad'] = median
        selected['joint5_stable'] = True
        selected['joint5_spread_deg'] = math.degrees(spread)

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
            source_frame = frame.copy()
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
                detection = {
                    'class': label, 'confidence': confidence,
                    'x': x1, 'y': y1, 'width': x2 - x1, 'height': y2 - y1,
                    'center_x': (x1 + x2) / 2.0, 'center_y': (y1 + y2) / 2.0,
                }
                color = (55, 200, 55) if label == 'normal' else (40, 60, 235)
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                cv2.putText(frame, f'{label} {confidence:.2f}', (int(x1), max(18, int(y1) - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
                if label == 'defect':
                    angle = self.estimate_joint5(
                        source_frame, (x1, y1, x2, y2),
                        (detection['center_x'], detection['center_y']))
                    if angle is not None:
                        rotated_box, axis, joint5, robot_x, robot_y = angle
                        detection.update({
                            'box_axis_deg': axis,
                            'joint5_raw_rad': joint5,
                            'robot_x': robot_x,
                            'robot_y': robot_y,
                        })
                        cv2.polylines(frame, [rotated_box.astype(int)], True,
                                      (255, 0, 255), 3)
                        cv2.putText(
                            frame, f'axis={axis:.1f} q5={joint5:.3f}rad',
                            (int(x1), min(frame.shape[0] - 8, int(y2) + 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 0, 255), 2)
                detections.append(detection)
            self.stabilize_best_defect_angle(detections)
            stable = next((d for d in detections if d.get('joint5_stable')), None)
            if stable is not None:
                cv2.putText(
                    frame,
                    f"STABLE joint5={float(stable['joint5_rad']):.3f}rad "
                    f"spread={float(stable['joint5_spread_deg']):.1f}deg",
                    (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 0, 255), 2)
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
        defects = [d for d in detections if d.get('class') == 'defect' and
                   d.get('joint5_stable')]
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
            f"conf={float(detection['confidence']):.2f}, "
            f"joint5={float(detection['joint5_rad']):.3f}rad")


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
