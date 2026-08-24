#!/usr/bin/env python3
"""Interactively create a seven-point camera-to-link0 homography file."""

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
            0.00, -0.33,
            0.00, 0.12,
            0.30, 0.07,
            0.29, -0.28,
            0.11, -0.17,
            0.20, -0.025,
            0.10, 0.075,
        ])
        self.declare_parameter(
            'output_file',
            '/root/omx_box_project_ws/integration/omx_box_system/calibration/'
            'omx_camera_homography_7point.yaml')
        self.declare_parameter('warning_mean_error_mm', 8.0)

        flat = list(self.get_parameter('reference_points_link0').value)
        if len(flat) != 14:
            raise ValueError('reference_points_link0 must contain exactly seven x,y pairs')
        self.reference_points = np.asarray(flat, dtype=np.float64).reshape((7, 2))
        self.output_file = os.path.abspath(str(self.get_parameter('output_file').value))
        self.warning_error_mm = float(self.get_parameter('warning_mean_error_mm').value)

        self.bridge = CvBridge()
        self.latest_image = None
        self.image_points = []
        self.homography = None
        self.errors_mm = None
        self.collecting = False
        self.window_name = 'OMX 7-Point Homography Calibration'

        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value), self.image_callback, 10)
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        self.timer = self.create_timer(0.03, self.display_image)

        self.get_logger().info('Measurement only: no robot command is published')
        self.get_logger().info(f'Output file: {self.output_file}')
        self.get_logger().info('Keys: c=start/restart, u=undo, q=quit')
        for index, point in enumerate(self.reference_points):
            self.get_logger().info(
                f'Click {index + 1}: link0 X={point[0]:.3f}m, Y={point[1]:.3f}m')

    def image_callback(self, message):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(message, 'bgr8')
        except Exception as error:  # cv_bridge raises several exception types
            self.get_logger().error(f'Image conversion failed: {error}')

    def mouse_callback(self, event, x, y, _flags, _userdata):
        if event != cv2.EVENT_LBUTTONDOWN or not self.collecting:
            return
        self.image_points.append((float(x), float(y)))
        index = len(self.image_points) - 1
        target = self.reference_points[index]
        self.get_logger().info(
            f'Captured {index + 1}/7: pixel=({x}, {y}) -> '
            f'link0=({target[0]:.3f}, {target[1]:.3f})')
        if len(self.image_points) == 7:
            self.compute_and_save()

    def compute_and_save(self):
        image = np.asarray(self.image_points, dtype=np.float64)
        matrix, _mask = cv2.findHomography(image, self.reference_points, 0)
        if matrix is None or matrix.shape != (3, 3) or not np.isfinite(matrix).all():
            self.get_logger().error('Homography calculation failed; press c and measure again')
            self.collecting = False
            return

        projected = cv2.perspectiveTransform(
            image.reshape((-1, 1, 2)), matrix).reshape((-1, 2))
        self.errors_mm = np.linalg.norm(
            projected - self.reference_points, axis=1) * 1000.0
        self.homography = matrix
        self.collecting = False

        output_dir = os.path.dirname(self.output_file)
        os.makedirs(output_dir, exist_ok=True)
        temporary_file = self.output_file + '.tmp'
        storage = cv2.FileStorage(temporary_file, cv2.FILE_STORAGE_WRITE)
        if not storage.isOpened():
            self.get_logger().error(f'Cannot write calibration: {temporary_file}')
            return
        storage.write('homography', self.homography)
        storage.write('image_points', image)
        storage.write('reference_points_link0', self.reference_points)
        storage.write('reprojection_errors_mm', self.errors_mm.reshape((-1, 1)))
        storage.release()
        os.replace(temporary_file, self.output_file)

        mean_error = float(np.mean(self.errors_mm))
        max_error = float(np.max(self.errors_mm))
        log = self.get_logger().warning if mean_error > self.warning_error_mm else self.get_logger().info
        log(f'Calibration saved: mean error={mean_error:.2f}mm, '
            f'max error={max_error:.2f}mm, file={self.output_file}')
        self.get_logger().info(
            'Restart the YOLO detector to apply this calibration; validate independent points first')

    def display_image(self):
        if self.latest_image is None:
            return
        display = self.latest_image.copy()
        for index, (x, y) in enumerate(self.image_points):
            cv2.circle(display, (int(x), int(y)), 6, (0, 255, 255), -1)
            cv2.putText(display, str(index + 1), (int(x) + 8, int(y) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        if self.collecting:
            index = len(self.image_points)
            point = self.reference_points[index]
            status = (f'CLICK {index + 1}/7  link0 X={point[0]:.3f} '
                      f'Y={point[1]:.3f} m')
        elif self.homography is not None:
            status = (f'SAVED  mean={np.mean(self.errors_mm):.2f}mm '
                      f'max={np.max(self.errors_mm):.2f}mm  c=redo')
        else:
            status = 'READY  c=start calibration'
        cv2.rectangle(display, (0, 0), (display.shape[1], 45), (0, 0, 0), -1)
        cv2.putText(display, status, (12, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (0, 255, 0), 2)
        cv2.imshow(self.window_name, display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            self.image_points = []
            self.homography = None
            self.errors_mm = None
            self.collecting = True
            self.get_logger().info('Calibration started; click points 1 through 7 in order')
        elif key == ord('u') and self.collecting and self.image_points:
            removed = self.image_points.pop()
            self.get_logger().info(f'Undid pixel ({removed[0]:.0f}, {removed[1]:.0f})')
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
