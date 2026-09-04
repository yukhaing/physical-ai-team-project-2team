#!/usr/bin/env python3
"""Track a Beagle-mounted ArUco marker and publish a taught unload target."""

from collections import deque
import json
import math
import os
from pathlib import Path

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class PlanarCalibration:
    """Pixel-to-link0 mapping shared with the unloading camera calibration."""

    def __init__(self, path):
        if not os.path.isfile(path):
            raise RuntimeError(f'unload calibration file not found: {path}')
        storage = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        if not storage.isOpened():
            raise RuntimeError(f'cannot open unload calibration: {path}')
        self.homography = storage.getNode('homography').mat()
        image = storage.getNode('image_points').mat()
        reference = storage.getNode('reference_points_link0').mat()
        polynomial = storage.getNode('polynomial_coefficients').mat()
        normalization = storage.getNode('pixel_normalization').mat()
        coordinate_model = storage.getNode('coordinate_model').string()
        storage.release()
        if (self.homography is None or self.homography.shape != (3, 3) or
                not np.all(np.isfinite(self.homography))):
            raise RuntimeError(f'invalid unload homography: {path}')
        if image is None or reference is None:
            raise RuntimeError(f'unload calibration points are missing: {path}')
        self.pixels = image.reshape((-1, 2)).astype(float)
        self.references = reference.reshape((-1, 2)).astype(float)
        if len(self.pixels) != len(self.references) or len(self.pixels) < 4:
            raise RuntimeError(f'invalid unload calibration point count: {path}')
        projected = cv2.perspectiveTransform(
            self.pixels.reshape((-1, 1, 2)), self.homography).reshape((-1, 2))
        self.residuals = self.references - projected
        self.polynomial = None
        self.normalization = None
        self.triangulation = None
        self.piecewise_coefficients = None
        self.model = 'homography_residual_fallback'
        if coordinate_model == 'piecewise_affine_v1':
            from scipy.spatial import Delaunay
            self.triangulation = Delaunay(self.pixels)
            self.piecewise_coefficients = [
                np.linalg.solve(
                    np.column_stack((np.ones(3), self.pixels[vertices])),
                    self.references[vertices])
                for vertices in self.triangulation.simplices]
            self.model = coordinate_model
        elif (polynomial is not None and polynomial.shape == (6, 2) and
              normalization is not None and normalization.size == 4 and
              np.all(np.isfinite(polynomial)) and
              np.all(np.isfinite(normalization))):
            self.polynomial = polynomial.astype(float)
            self.normalization = normalization.reshape(4).astype(float)
            self.model = 'quadratic_v1'

    def world(self, point):
        pixel = np.asarray(point, dtype=float)
        if self.triangulation is not None:
            simplex = int(self.triangulation.find_simplex(pixel))
            if simplex < 0:
                return np.asarray([float('nan'), float('nan')])
            return (np.asarray([1.0, pixel[0], pixel[1]]) @
                    self.piecewise_coefficients[simplex])
        if self.polynomial is not None:
            cx, cy, sx, sy = self.normalization
            u, v = (pixel[0] - cx) / sx, (pixel[1] - cy) / sy
            return np.asarray([1.0, u, v, u * u, u * v, v * v]) @ self.polynomial
        transformed = self.homography @ np.asarray([pixel[0], pixel[1], 1.0])
        if abs(float(transformed[2])) < 1.0e-12:
            return np.asarray([float('nan'), float('nan')])
        raw = transformed[:2] / transformed[2]
        distance = np.linalg.norm(self.pixels - pixel, axis=1)
        nearest = int(np.argmin(distance))
        if distance[nearest] < 1.0:
            return raw + self.residuals[nearest]
        weights = 1.0 / (distance * distance + 1.0e-6)
        return raw + (weights[:, None] * self.residuals).sum(axis=0) / weights.sum()


class UnloadMarkerTarget(Node):
    """Convert a stable Beagle marker pose into the robot's unload pick target."""

    def __init__(self):
        super().__init__('unload_marker_target')
        defaults = {
            'image_topic': '/unload_camera/image_raw',
            'target_topic': '/unload_omx/vision_target',
            'raw_target_topic': '/unload_omx/vision_raw_target',
            'annotated_image_topic': '/unload_vision/annotated_image',
            'status_topic': '/unload_vision/status',
            'base_frame': 'link0',
            'calibration_file': '/root/omx_box_project_ws/integration/yeongjin_gui/runtime/calibration/unload_active.yaml',
            'teach_profile_file': '/root/omx_box_project_ws/integration/yeongjin_gui/runtime/calibration/unload_source_teach.yaml',
            'teaching_mode': False,
            'landmark_mode': 'tray',
            'dictionary_name': 'DICT_4X4_50',
            'marker_id': 0,
            'tray_gray_threshold': 55,
            'tray_roi_min_y': 210,
            'tray_min_area': 10000.0,
            'tray_max_area': 30000.0,
            'tray_min_size': 100.0,
            'tray_max_size': 220.0,
            'minimum_marker_radius': 0.05,
            'maximum_marker_radius': 0.40,
            'minimum_reach_radius': 0.10,
            'maximum_reach_radius': 0.29,
            'stability_samples': 7,
            'maximum_xy_spread': 0.005,
            'maximum_angle_spread': 0.035,
            'maximum_marker_shift': 0.030,
            'maximum_marker_rotation': 0.087266463,
            'minimum_joint5': -0.80,
            'maximum_joint5': 0.80,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        dictionary_name = str(self.p('dictionary_name'))
        if not hasattr(cv2.aruco, dictionary_name):
            raise RuntimeError(f'unknown ArUco dictionary: {dictionary_name}')
        dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, dictionary_name))
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        self.calibration = PlanarCalibration(str(self.p('calibration_file')))
        self.bridge = CvBridge()
        self.samples = deque(maxlen=max(3, int(self.p('stability_samples'))))
        self.taught_source_xy = None
        self.reference_marker_xy = None
        self.reference_marker_angle = None
        self.taught_joint5 = 0.0
        if not bool(self.p('teaching_mode')):
            self.load_teach_profile()
        else:
            self.get_logger().info('teaching_mode=true; marker pose is published raw')
        self.target_pub = self.create_publisher(
            PoseStamped, str(self.p('target_topic')), 10)
        self.raw_pub = self.create_publisher(
            PoseStamped, str(self.p('raw_target_topic')), 10)
        self.image_pub = self.create_publisher(
            Image, str(self.p('annotated_image_topic')), 2)
        self.status_pub = self.create_publisher(
            String, str(self.p('status_topic')), 10)
        self.create_subscription(Image, str(self.p('image_topic')), self.on_image, 2)
        self.get_logger().info(
            f'ArUco unload marker ready; dictionary={dictionary_name}, '
            f'id={int(self.p("marker_id"))}, calibration={self.calibration.model}')

    def p(self, name):
        return self.get_parameter(name).value

    @staticmethod
    def wrap(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def load_teach_profile(self):
        path = Path(str(self.p('teach_profile_file')))
        if not path.is_file():
            self.get_logger().warning(f'marker teach profile is absent: {path}')
            return
        try:
            import yaml
            data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
            taught = np.asarray(data['taught_source_xy'], dtype=float)
            reference = np.asarray(data.get('reference_landmark_xy',
                                            data.get('reference_marker_xy')), dtype=float)
            angle = float(data['reference_marker_angle'])
            joint5 = float(data.get('taught_joint5', 0.0))
            if (taught.shape != (2,) or reference.shape != (2,) or
                    not np.all(np.isfinite(taught)) or
                    not np.all(np.isfinite(reference)) or
                    not math.isfinite(angle) or not math.isfinite(joint5)):
                raise ValueError('marker teaching values are invalid')
            self.taught_source_xy = taught
            self.reference_marker_xy = reference
            self.reference_marker_angle = angle
            self.taught_joint5 = joint5
            self.get_logger().info(
                f'loaded marker teach profile; robot={taught.tolist()}, '
                f'marker={reference.tolist()}, angle={math.degrees(angle):.2f}deg')
        except Exception as error:  # noqa: BLE001 - configuration boundary
            self.get_logger().error(f'cannot load marker teach profile {path}: {error}')

    def marker_pose(self, corners, ids):
        if ids is None:
            return None
        matches = np.flatnonzero(ids.reshape(-1) == int(self.p('marker_id')))
        if len(matches) != 1:
            return None
        pixel_corners = np.asarray(corners[int(matches[0])], dtype=float).reshape(4, 2)
        center_pixel = np.mean(pixel_corners, axis=0)
        center = self.calibration.world(center_pixel)
        edge_start = self.calibration.world(pixel_corners[0])
        edge_end = self.calibration.world(pixel_corners[1])
        if not (np.all(np.isfinite(center)) and np.all(np.isfinite(edge_start)) and
                np.all(np.isfinite(edge_end))):
            return None
        radius = float(np.linalg.norm(center - np.asarray([-0.00999, 0.0])))
        if not float(self.p('minimum_marker_radius')) <= radius <= float(
                self.p('maximum_marker_radius')):
            return None
        angle = math.atan2(float(edge_end[1] - edge_start[1]),
                           float(edge_end[0] - edge_start[0]))
        return center_pixel, pixel_corners, center, angle

    def tray_pose(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mask = (gray < int(self.p('tray_gray_threshold'))).astype(np.uint8) * 255
        mask[:max(0, int(self.p('tray_roi_min_y'))), :] = 0
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        height, width = frame.shape[:2]
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if not float(self.p('tray_min_area')) <= area <= float(
                    self.p('tray_max_area')):
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if not (float(self.p('tray_min_size')) <= w <= float(self.p('tray_max_size')) and
                    float(self.p('tray_min_size')) <= h <= float(self.p('tray_max_size'))):
                continue
            if x <= 5 or y <= int(self.p('tray_roi_min_y')) or x + w >= width - 5 or y + h >= height - 5:
                continue
            rectangle = cv2.minAreaRect(contour)
            center_pixel = np.asarray(rectangle[0], dtype=float)
            center = self.calibration.world(center_pixel)
            if not np.all(np.isfinite(center)):
                continue
            radius = float(np.linalg.norm(center - np.asarray([-0.00999, 0.0])))
            if not float(self.p('minimum_marker_radius')) <= radius <= float(
                    self.p('maximum_marker_radius')):
                continue
            box = cv2.boxPoints(rectangle)
            candidates.append((area, center_pixel, box, center))
        if not candidates:
            return None
        _area, center_pixel, box, center = max(candidates, key=lambda item: item[0])
        # The tray silhouette is close to square, so minAreaRect angle can flip by
        # 90 degrees. Docking correction currently needs translation only.
        return center_pixel, box, center, 0.0

    def stable_pose(self, sample):
        if sample is None:
            self.samples.clear()
            return None
        self.samples.append(sample)
        if len(self.samples) < self.samples.maxlen:
            return None
        points = np.asarray([item[0] for item in self.samples], dtype=float)
        center = np.median(points, axis=0)
        xy_spread = float(np.max(np.linalg.norm(points - center, axis=1)))
        angles = np.asarray([item[1] for item in self.samples], dtype=float)
        angle = math.atan2(float(np.mean(np.sin(angles))),
                           float(np.mean(np.cos(angles))))
        errors = np.asarray([self.wrap(value - angle) for value in angles])
        angle_spread = float(np.ptp(errors))
        if (xy_spread > float(self.p('maximum_xy_spread')) or
                angle_spread > float(self.p('maximum_angle_spread'))):
            return None
        return center, angle, xy_spread, angle_spread

    def pose_message(self, header, xy, angle, raw):
        message = PoseStamped()
        message.header = header
        message.header.frame_id = str(self.p('base_frame'))
        message.pose.position.x = float(xy[0])
        message.pose.position.y = float(xy[1])
        message.pose.position.z = 0.0
        if raw:
            message.pose.orientation.z = math.sin(angle / 2.0)
        else:
            message.pose.orientation.x = math.sin(angle / 2.0)
        message.pose.orientation.w = math.cos(angle / 2.0)
        return message

    def publish_stable(self, stable, center_pixel, header):
        marker_xy, marker_angle, xy_spread, angle_spread = stable
        self.raw_pub.publish(self.pose_message(header, marker_xy, marker_angle, True))
        payload = {
            'landmark_type': str(self.p('landmark_mode')).lower(),
            'marker_id': int(self.p('marker_id')),
            'pixel_x': float(center_pixel[0]), 'pixel_y': float(center_pixel[1]),
            'marker_x': float(marker_xy[0]), 'marker_y': float(marker_xy[1]),
            'marker_angle_deg': math.degrees(marker_angle),
            'xy_spread_mm': xy_spread * 1000.0,
            'angle_spread_deg': math.degrees(angle_spread),
        }
        if (self.taught_source_xy is None or self.reference_marker_xy is None or
                self.reference_marker_angle is None):
            payload['state'] = 'TEACH_REQUIRED'
            self.status_pub.publish(String(data=json.dumps(payload)))
            return
        marker_shift = marker_xy - self.reference_marker_xy
        angle_delta = self.wrap(marker_angle - self.reference_marker_angle)
        if (float(np.linalg.norm(marker_shift)) > float(self.p('maximum_marker_shift')) or
                abs(angle_delta) > float(self.p('maximum_marker_rotation'))):
            payload.update({
                'state': 'OUT_OF_RANGE',
                'marker_shift_mm': float(np.linalg.norm(marker_shift) * 1000.0),
                'marker_rotation_deg': math.degrees(angle_delta)})
            self.status_pub.publish(String(data=json.dumps(payload)))
            return
        cosine, sine = math.cos, math.sin
        reference_rotation_inverse = np.asarray([
            [cosine(-self.reference_marker_angle), -sine(-self.reference_marker_angle)],
            [sine(-self.reference_marker_angle), cosine(-self.reference_marker_angle)]])
        current_rotation = np.asarray([
            [cosine(marker_angle), -sine(marker_angle)],
            [sine(marker_angle), cosine(marker_angle)]])
        marker_to_pick = reference_rotation_inverse @ (
            self.taught_source_xy - self.reference_marker_xy)
        target_xy = marker_xy + current_rotation @ marker_to_pick
        radius = float(np.linalg.norm(target_xy - np.asarray([-0.00999, 0.0])))
        joint5 = self.wrap(self.taught_joint5 + angle_delta)
        if (not float(self.p('minimum_reach_radius')) <= radius <=
                float(self.p('maximum_reach_radius')) or
                not float(self.p('minimum_joint5')) <= joint5 <=
                float(self.p('maximum_joint5'))):
            payload['state'] = 'UNSAFE_TARGET'
            self.status_pub.publish(String(data=json.dumps(payload)))
            return
        self.target_pub.publish(self.pose_message(header, target_xy, joint5, False))
        payload.update({
            'state': 'STABLE', 'robot_x': float(target_xy[0]),
            'robot_y': float(target_xy[1]), 'joint5_rad': joint5,
            'marker_shift_x_mm': float(marker_shift[0] * 1000.0),
            'marker_shift_y_mm': float(marker_shift[1] * 1000.0),
            'marker_rotation_deg': math.degrees(angle_delta),
            'z_source': 'coordinator_fixed_loading_place'})
        self.status_pub.publish(String(data=json.dumps(payload)))

    def on_image(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            annotated = frame.copy()
            mode = str(self.p('landmark_mode')).lower()
            if mode not in ('tray', 'aruco', 'auto'):
                raise RuntimeError(f'unsupported landmark_mode: {mode}')
            detected = None
            landmark_type = 'tray'
            if mode in ('aruco', 'auto'):
                corners, ids, _rejected = self.detector.detectMarkers(frame)
                detected = self.marker_pose(corners, ids)
                landmark_type = 'aruco'
            if detected is None and mode in ('tray', 'auto'):
                detected = self.tray_pose(frame)
                landmark_type = 'tray'
            stable = None
            center_pixel = None
            if detected is not None:
                center_pixel, pixel_corners, marker_xy, marker_angle = detected
                cv2.polylines(
                    annotated, [np.round(pixel_corners).astype(int)], True,
                    (40, 220, 40), 3)
                cv2.circle(annotated, tuple(np.round(center_pixel).astype(int)),
                           6, (0, 255, 255), -1)
                stable = self.stable_pose((marker_xy, marker_angle))
            else:
                self.stable_pose(None)
            if stable is not None:
                self.publish_stable(stable, center_pixel, message.header)
                marker_xy, angle, spread, _angle_spread = stable
                text = (f'{landmark_type.upper()} X={marker_xy[0]:+.3f} '
                        f'Y={marker_xy[1]:+.3f} A={math.degrees(angle):+.1f} '
                        f'spread={spread*1000.0:.1f}mm')
                color = (0, 255, 255)
            else:
                state = ('LANDMARK_NOT_FOUND' if detected is None else
                         f'STABILIZING {len(self.samples)}/{self.samples.maxlen}')
                self.status_pub.publish(String(data=json.dumps({
                    'state': state, 'landmark_type': landmark_type,
                    'marker_id': int(self.p('marker_id'))})))
                text, color = state, (0, 160, 255)
            cv2.putText(annotated, text, (8, 28), cv2.FONT_HERSHEY_SIMPLEX,
                        0.56, color, 2)
            output = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            output.header = message.header
            self.image_pub.publish(output)
        except Exception as error:  # noqa: BLE001 - camera callback boundary
            self.samples.clear()
            self.status_pub.publish(String(data=json.dumps({
                'state': 'ERROR', 'message': str(error)})))
            self.get_logger().error(f'unload marker callback failed: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = UnloadMarkerTarget()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
