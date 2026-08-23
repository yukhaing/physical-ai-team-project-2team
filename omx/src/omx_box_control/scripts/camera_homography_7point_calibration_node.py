#!/usr/bin/env python3
"""Measurement-only seven-point camera-to-link0 homography calibration tool."""

import os

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraHomography7PointCalibration(Node):
    def __init__(self):
        super().__init__('camera_homography_7point_calibration')
        self.declare_parameter('image_topic', '/camera1/image_raw')
        self.declare_parameter('reference_points_link0', [
            0.00, -0.33, 0.00, 0.12, 0.30, 0.07, 0.29, -0.28,
            0.11, -0.17, 0.20, -0.025, 0.10, 0.075])
        self.declare_parameter('output_file', '/tmp/omx_camera_homography_7point.yaml')
        self.declare_parameter('warning_mean_error_mm', 8.0)
        flat = list(self.get_parameter('reference_points_link0').value)
        if len(flat) != 14:
            raise ValueError('reference_points_link0 must contain exactly seven x,y pairs')
        self.reference_points = np.asarray(flat, dtype=np.float64).reshape((7, 2))
        self.output_file = os.path.abspath(str(self.get_parameter('output_file').value))
        self.warning_error_mm = float(self.get_parameter('warning_mean_error_mm').value)
        self.bridge, self.latest_image = CvBridge(), None
        self.image_points, self.validation_points = [], []
        self.homography = self.errors_mm = None
        self.collecting = False
        self.window_name = 'OMX 7-Point Homography Calibration'
        self.create_subscription(Image, str(self.get_parameter('image_topic').value), self.on_image, 10)
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.on_click)
        self.timer = self.create_timer(0.03, self.draw)
        self.get_logger().info('Measurement only: no robot target is published. Press c to calibrate.')

    def on_image(self, message):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(message, 'bgr8')
        except Exception as error:
            self.get_logger().error(f'Image conversion failed: {error}')

    def on_click(self, event, x, y, _flags, _userdata):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.collecting:
            self.image_points.append((float(x), float(y)))
            point = self.reference_points[len(self.image_points) - 1]
            self.get_logger().info(f'Captured {len(self.image_points)}/7: pixel=({x}, {y}) -> link0=({point[0]:.3f}, {point[1]:.3f})')
            if len(self.image_points) == 7:
                self.compute_and_save()
        elif self.homography is not None:
            pixel = np.asarray([[[float(x), float(y)]]], dtype=np.float64)
            world_x, world_y = cv2.perspectiveTransform(pixel, self.homography)[0, 0]
            self.validation_points.append((x, y, world_x, world_y))
            self.get_logger().info(f'Validation only: link0 X={world_x:.4f}m, Y={world_y:.4f}m')

    def compute_and_save(self):
        image = np.asarray(self.image_points, dtype=np.float64)
        matrix, _ = cv2.findHomography(image, self.reference_points, 0)
        if matrix is None or matrix.shape != (3, 3) or not np.isfinite(matrix).all():
            self.collecting = False
            self.get_logger().error('Homography calculation failed; press c and retry')
            return
        projected = cv2.perspectiveTransform(image.reshape((-1, 1, 2)), matrix).reshape((-1, 2))
        self.errors_mm = np.linalg.norm(projected - self.reference_points, axis=1) * 1000.0
        self.homography, self.collecting = matrix, False
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
        temporary = self.output_file + '.tmp'
        storage = cv2.FileStorage(temporary, cv2.FILE_STORAGE_WRITE)
        if not storage.isOpened():
            self.get_logger().error(f'Cannot write calibration: {temporary}')
            return
        storage.write('homography', self.homography)
        storage.write('image_points', image)
        storage.write('reference_points_link0', self.reference_points)
        storage.write('reprojection_errors_mm', self.errors_mm.reshape((-1, 1)))
        storage.release()
        os.replace(temporary, self.output_file)
        mean_error, maximum_error = float(np.mean(self.errors_mm)), float(np.max(self.errors_mm))
        log = self.get_logger().warning if mean_error > self.warning_error_mm else self.get_logger().info
        log(f'Calibration saved: mean={mean_error:.2f}mm, max={maximum_error:.2f}mm, file={self.output_file}')

    def draw(self):
        if self.latest_image is None:
            return
        image = self.latest_image.copy()
        for index, (x, y) in enumerate(self.image_points):
            cv2.circle(image, (int(x), int(y)), 6, (0, 255, 255), -1)
            cv2.putText(image, str(index + 1), (int(x) + 8, int(y) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        status = f'CLICK {len(self.image_points) + 1}/7' if self.collecting else 'READY: c=calibrate, click=validate, q=quit'
        cv2.putText(image, status, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        cv2.imshow(self.window_name, image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            self.image_points, self.validation_points, self.homography, self.errors_mm = [], [], None, None
            self.collecting = True
        elif key == ord('u') and self.collecting and self.image_points:
            self.image_points.pop()
        elif key == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraHomography7PointCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
