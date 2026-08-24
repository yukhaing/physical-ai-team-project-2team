#!/usr/bin/env python3
"""Select planar OMX targets from a monocular camera using a homography."""

import os
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker


class CameraHomographyTarget(Node):
    def __init__(self):
        super().__init__('camera_homography_target')
        self.declare_parameter('image_topic', '/camera1/image_raw')
        self.declare_parameter('base_frame', 'link0')
        self.declare_parameter('target_z', 0.10)
        self.declare_parameter('reference_points_link0',
                               [0.12, -0.10, 0.28, -0.10, 0.28, 0.10, 0.12, 0.10])
        self.declare_parameter('target_pose_topic', '/camera_box_target')
        self.declare_parameter('marker_topic', '/camera_box_marker')
        self.declare_parameter('calibration_file', '/tmp/omx_camera_homography.yaml')
        self.declare_parameter('show_window', True)
        flat = list(self.get_parameter('reference_points_link0').value)
        if len(flat) < 8 or len(flat) % 2:
            raise ValueError('reference_points_link0 must contain at least four x,y pairs')
        self.reference_points = np.asarray(flat, dtype=np.float64).reshape((-1, 2))
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.target_z = float(self.get_parameter('target_z').value)
        self.calibration_file = str(self.get_parameter('calibration_file').value)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_points = []
        self.homography = None
        self.mode = 'target'
        self.window_name = 'OMX Homography Target'
        self.pose_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('target_pose_topic').value), 10)
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(
            Marker, str(self.get_parameter('marker_topic').value), marker_qos)
        self.image_sub = self.create_subscription(
            Image, str(self.get_parameter('image_topic').value), self.image_callback, 10)
        self.load_calibration()
        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.setMouseCallback(self.window_name, self.mouse_callback)
            self.timer = self.create_timer(0.03, self.display_image)
        self.get_logger().info('Keys: c=calibrate, r=reset, q=close window')
        self.get_logger().info('Preview only: this node does not command the robot')

    def image_callback(self, message):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(message, 'bgr8')
        except Exception as error:
            self.get_logger().error(f'Image conversion failed: {error}')

    def mouse_callback(self, event, x, y, _flags, _userdata):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.mode == 'calibration':
            self.image_points.append((float(x), float(y)))
            index = len(self.image_points) - 1
            point = self.reference_points[index]
            self.get_logger().info(
                f'Reference {index + 1}/{len(self.reference_points)}: pixel=({x}, {y}) '
                f'-> link0=({point[0]:.3f}, {point[1]:.3f})')
            if len(self.image_points) == len(self.reference_points):
                self.compute_homography()
        elif self.homography is None:
            self.get_logger().warning("No calibration. Press 'c' and click reference points")
        else:
            self.publish_target(x, y)

    def compute_homography(self):
        image = np.asarray(self.image_points, dtype=np.float64)
        matrix, _ = cv2.findHomography(image, self.reference_points, 0)
        if matrix is None or not np.all(np.isfinite(matrix)):
            self.get_logger().error('Homography calculation failed; reset and retry')
            return
        self.homography = matrix
        self.mode = 'target'
        self.save_calibration()
        projected = cv2.perspectiveTransform(image.reshape((-1, 1, 2)), matrix).reshape((-1, 2))
        error = np.linalg.norm(projected - self.reference_points, axis=1)
        self.get_logger().info(
            f'Calibration complete; mean reference error={np.mean(error) * 1000.0:.1f} mm')

    def publish_target(self, pixel_x, pixel_y):
        pixel = np.asarray([[[float(pixel_x), float(pixel_y)]]], dtype=np.float64)
        x, y = cv2.perspectiveTransform(pixel, self.homography)[0, 0]
        if not np.isfinite(x) or not np.isfinite(y):
            self.get_logger().error('Selected pixel produced an invalid target')
            return
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.position.z, pose.pose.orientation.w = self.target_z, 1.0
        self.pose_pub.publish(pose)
        marker = Marker()
        marker.header, marker.pose = pose.header, pose.pose
        marker.ns, marker.id = 'camera_box_target', 0
        marker.type, marker.action = Marker.SPHERE, Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = 0.025
        marker.color.r, marker.color.g, marker.color.a = 1.0, 0.3, 1.0
        self.marker_pub.publish(marker)
        self.get_logger().info(
            f'pixel=({pixel_x}, {pixel_y}) -> {self.base_frame}: '
            f'x={x:.3f}, y={y:.3f}, z={self.target_z:.3f} (preview only)')

    def display_image(self):
        if self.latest_image is None:
            return
        display = self.latest_image.copy()
        for index, (x, y) in enumerate(self.image_points):
            cv2.circle(display, (int(x), int(y)), 6, (0, 255, 255), -1)
            cv2.putText(display, str(index + 1), (int(x) + 8, int(y) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        if self.mode == 'calibration':
            status = f'CALIBRATION: reference {len(self.image_points) + 1}/{len(self.reference_points)}'
        elif self.homography is not None:
            status = 'TARGET: click a box point'
        else:
            status = "UNCALIBRATED: press 'c'"
        cv2.putText(display, status, (15, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 0), 2)
        cv2.imshow(self.window_name, display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            self.image_points, self.mode = [], 'calibration'
            self.get_logger().info('Click reference points in YAML order')
        elif key == ord('r'):
            self.image_points, self.homography, self.mode = [], None, 'target'
            self.get_logger().info('Calibration reset')
        elif key == ord('q'):
            cv2.destroyWindow(self.window_name)
            self.show_window = False
            self.timer.cancel()

    def save_calibration(self):
        storage = cv2.FileStorage(self.calibration_file, cv2.FILE_STORAGE_WRITE)
        if not storage.isOpened():
            self.get_logger().error(f'Cannot write calibration: {self.calibration_file}')
            return
        storage.write('homography', self.homography)
        storage.write('image_points', np.asarray(self.image_points, dtype=np.float64))
        storage.write('reference_points_link0', self.reference_points)
        storage.release()
        self.get_logger().info(f'Saved calibration to {self.calibration_file}')

    def load_calibration(self):
        if not os.path.isfile(self.calibration_file):
            return
        storage = cv2.FileStorage(self.calibration_file, cv2.FILE_STORAGE_READ)
        matrix = storage.getNode('homography').mat() if storage.isOpened() else None
        image = storage.getNode('image_points').mat() if storage.isOpened() else None
        storage.release()
        if matrix is not None and matrix.shape == (3, 3) and np.all(np.isfinite(matrix)):
            self.homography = matrix
            if image is not None:
                self.image_points = [tuple(point) for point in image.reshape((-1, 2))]
            self.get_logger().info(f'Loaded calibration from {self.calibration_file}')

    def destroy_node(self):
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraHomographyTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
