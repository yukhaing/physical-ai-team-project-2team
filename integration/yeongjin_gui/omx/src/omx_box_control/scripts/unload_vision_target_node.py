#!/usr/bin/env python3
"""Detect and stabilize the Beagle box in unloading-OMX link0 coordinates."""

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
    """Load the piecewise/quadratic/homography calibration used by OMX cameras."""

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


class UnloadVisionTarget(Node):
    """Publish only a geometrically valid and temporally stable Beagle-box target."""

    def __init__(self):
        super().__init__('unload_vision_target')
        defaults = {
            'image_topic': '/unload_camera/image_raw',
            'target_topic': '/unload_omx/vision_target',
            'raw_target_topic': '/unload_omx/vision_raw_target',
            'annotated_image_topic': '/unload_vision/annotated_image',
            'status_topic': '/unload_vision/status',
            'base_frame': 'link0',
            'model_path': '/root/omx_box_project_ws/integration/omx_box_system/models/box_defect_best.pt',
            'calibration_file': '/root/omx_box_project_ws/integration/yeongjin_gui/runtime/calibration/unload_active.yaml',
            'confidence_threshold': 0.40,
            'device': 'cpu',
            'target_classes': ['defect', 'normal'],
            'expected_source_xy': [-0.02621, -0.17335],
            'teach_profile_file': '/root/omx_box_project_ws/integration/yeongjin_gui/runtime/calibration/unload_source_teach.yaml',
            'teaching_mode': False,
            'maximum_source_offset': 0.10,
            'minimum_reach_radius': 0.10,
            'maximum_reach_radius': 0.29,
            'stability_samples': 5,
            'maximum_xy_spread': 0.012,
            'maximum_angle_spread': 0.18,
            'require_angle': False,
            'fallback_joint5': 0.0,
            'minimum_joint5': -0.80,
            'maximum_joint5': 0.80,
            'joint5_offset_deg': 2.00255,
            'angle_min_contour_area': 1000.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        try:
            from ultralytics import YOLO
            self.model = YOLO(str(self.p('model_path')))
        except Exception as error:
            raise RuntimeError(f'unable to load unload YOLO model: {error}') from error
        self.calibration = PlanarCalibration(str(self.p('calibration_file')))
        self.taught_source_xy = None
        self.reference_camera_xy = None
        if not bool(self.p('teaching_mode')):
            self.load_teach_profile()
        else:
            self.get_logger().info('teaching_mode=true; publishing unrestricted reachable raw target')
        sample_count = max(3, int(self.p('stability_samples')))
        self.samples = deque(maxlen=sample_count)
        self.bridge = CvBridge()
        self.pose_pub = self.create_publisher(PoseStamped, str(self.p('target_topic')), 10)
        self.raw_pose_pub = self.create_publisher(
            PoseStamped, str(self.p('raw_target_topic')), 10)
        self.image_pub = self.create_publisher(
            Image, str(self.p('annotated_image_topic')), 2)
        self.status_pub = self.create_publisher(String, str(self.p('status_topic')), 10)
        self.create_subscription(Image, str(self.p('image_topic')), self.on_image, 2)
        self.get_logger().info(
            f'unload vision ready; calibration={self.calibration.model}, '
            f'samples={sample_count}')

    def p(self, name):
        return self.get_parameter(name).value

    def load_teach_profile(self):
        path = Path(str(self.p('teach_profile_file')))
        if not path.is_file():
            self.get_logger().warning(
                f'unload source teach profile is absent: {path}; '
                'raw target is available for teaching')
            return
        try:
            import yaml
            data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
            taught = np.asarray(data['taught_source_xy'], dtype=float)
            reference = np.asarray(data['reference_camera_xy'], dtype=float)
            if taught.shape != (2,) or reference.shape != (2,) or not (
                    np.all(np.isfinite(taught)) and np.all(np.isfinite(reference))):
                raise ValueError('XY entries must each contain two finite values')
            self.taught_source_xy = taught
            self.reference_camera_xy = reference
            self.get_logger().info(
                f'loaded unload source teach profile; taught={taught.tolist()}, '
                f'camera_reference={reference.tolist()}')
        except Exception as error:  # noqa: BLE001 - configuration boundary
            self.get_logger().error(
                f'cannot load unload source teach profile {path}: {error}')

    @staticmethod
    def normalize_half_turn(angle):
        return (angle + math.pi / 2.0) % math.pi - math.pi / 2.0

    def source_is_valid(self, xy):
        if not np.all(np.isfinite(xy)):
            return False
        expected = (self.reference_camera_xy if self.reference_camera_xy is not None else
                    np.asarray(self.p('expected_source_xy'), dtype=float))
        if np.linalg.norm(xy - expected) > float(self.p('maximum_source_offset')):
            return False
        center = np.asarray([-0.00999, 0.0])
        radius = float(np.linalg.norm(xy - center))
        return (float(self.p('minimum_reach_radius')) <= radius <=
                float(self.p('maximum_reach_radius')))

    def estimate_joint5(self, frame, bounds, center):
        x1, y1, x2, y2 = [int(round(value)) for value in bounds]
        xa, ya = max(0, x1 - 8), max(0, y1 - 8)
        xb, yb = min(frame.shape[1], x2 + 8), min(frame.shape[0], y2 + 8)
        if xb <= xa or yb <= ya:
            return None, None
        hsv = cv2.cvtColor(frame[ya:yb, xa:xb], cv2.COLOR_BGR2HSV)
        mask = ((hsv[:, :, 1] > 18) & (hsv[:, :, 1] < 150) &
                (hsv[:, :, 2] < 245) & (hsv[:, :, 0] > 4) &
                (hsv[:, :, 0] < 35)).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [item for item in contours if cv2.contourArea(item) >
                    float(self.p('angle_min_contour_area'))]
        if not contours:
            return None, None
        rectangle = cv2.boxPoints(cv2.minAreaRect(max(contours, key=cv2.contourArea)))
        rectangle[:, 0] += xa
        rectangle[:, 1] += ya
        edges = [(np.linalg.norm(rectangle[(i + 1) % 4] - rectangle[i]),
                  rectangle[i], rectangle[(i + 1) % 4]) for i in range(4)]
        _length, start, end = max(edges, key=lambda item: item[0])
        world_start, world_end = self.calibration.world(start), self.calibration.world(end)
        if not np.all(np.isfinite(world_start)) or not np.all(np.isfinite(world_end)):
            return rectangle, None
        axis = math.atan2(world_end[1] - world_start[1], world_end[0] - world_start[0])
        robot = self.calibration.world(center)
        joint1 = math.atan2(float(robot[1]), float(robot[0]) + 0.01125)
        joint5 = self.normalize_half_turn(
            joint1 + math.pi / 2.0 - axis + math.radians(float(self.p('joint5_offset_deg'))))
        if not float(self.p('minimum_joint5')) <= joint5 <= float(self.p('maximum_joint5')):
            return rectangle, None
        return rectangle, joint5

    def select_candidate(self, frame, result):
        expected = self.reference_camera_xy
        accepted = set(str(value) for value in self.p('target_classes'))
        candidates = []
        for box in result.boxes:
            bounds = [float(value) for value in box.xyxy[0].tolist()]
            label = str(result.names[int(box.cls[0].item())])
            confidence = float(box.conf[0].item())
            x1, y1, x2, y2 = bounds
            center = np.asarray([(x1 + x2) / 2.0, (y1 + y2) / 2.0])
            xy = self.calibration.world(center)
            color = (70, 70, 220)
            center_axis = np.asarray([-0.00999, 0.0])
            radius = float(np.linalg.norm(xy - center_axis)) if np.all(np.isfinite(xy)) else 0.0
            raw_reachable = (np.all(np.isfinite(xy)) and
                             float(self.p('minimum_reach_radius')) <= radius <=
                             float(self.p('maximum_reach_radius')))
            if label in accepted and raw_reachable and (
                    self.taught_source_xy is None or self.source_is_valid(xy)):
                rectangle, joint5 = self.estimate_joint5(frame, bounds, center)
                if joint5 is not None or not bool(self.p('require_angle')):
                    if joint5 is None:
                        joint5 = float(self.p('fallback_joint5'))
                    score = (-confidence if expected is None else
                             float(np.linalg.norm(xy - expected)) - confidence * 0.02)
                    candidates.append((score, confidence, center, xy, joint5, bounds, rectangle))
                    color = (40, 200, 40)
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            text_xy = 'out' if not np.all(np.isfinite(xy)) else f'{xy[0]:+.3f},{xy[1]:+.3f}'
            cv2.putText(frame, f'{label} {confidence:.2f} {text_xy}',
                        (int(x1), max(18, int(y1) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 2)
        return min(candidates, key=lambda item: item[0]) if candidates else None

    def stable_target(self, sample):
        if sample is None:
            self.samples.clear()
            return None
        self.samples.append(sample)
        if len(self.samples) < self.samples.maxlen:
            return None
        xy = np.asarray([item['xy'] for item in self.samples], dtype=float)
        center = np.median(xy, axis=0)
        spread = float(np.max(np.linalg.norm(xy - center, axis=1)))
        if spread > float(self.p('maximum_xy_spread')):
            return None
        angles = np.asarray([item['joint5'] for item in self.samples], dtype=float)
        phase = angles * 2.0
        angle = 0.5 * math.atan2(float(np.mean(np.sin(phase))),
                                 float(np.mean(np.cos(phase))))
        errors = np.asarray([self.normalize_half_turn(value - angle) for value in angles])
        if float(np.ptp(errors)) > float(self.p('maximum_angle_spread')):
            return None
        return center, angle, spread, float(np.ptp(errors))

    def publish_target(self, stable, pixel, confidence, header):
        raw_xy, joint5, xy_spread, angle_spread = stable
        raw_pose = PoseStamped()
        raw_pose.header = header
        raw_pose.header.frame_id = str(self.p('base_frame'))
        raw_pose.pose.position.x = float(raw_xy[0])
        raw_pose.pose.position.y = float(raw_xy[1])
        raw_pose.pose.orientation.x = math.sin(joint5 / 2.0)
        raw_pose.pose.orientation.w = math.cos(joint5 / 2.0)
        self.raw_pose_pub.publish(raw_pose)
        if self.taught_source_xy is None or self.reference_camera_xy is None:
            payload = {
                'state': 'TEACH_REQUIRED', 'pixel_x': float(pixel[0]),
                'pixel_y': float(pixel[1]), 'camera_x': float(raw_xy[0]),
                'camera_y': float(raw_xy[1]), 'confidence': confidence,
                'xy_spread_mm': xy_spread * 1000.0,
            }
            self.status_pub.publish(String(data=json.dumps(payload)))
            return
        xy = self.taught_source_xy + (raw_xy - self.reference_camera_xy)
        pose = PoseStamped()
        pose.header = header
        pose.header.frame_id = str(self.p('base_frame'))
        pose.pose.position.x = float(xy[0])
        pose.pose.position.y = float(xy[1])
        # The monocular calibration owns XY only. Z deliberately carries no
        # motion target; the coordinator uses the loading OMX's validated
        # Beagle place approach/release heights instead.
        pose.pose.position.z = 0.0
        # The OMX tool roll is carried as an X-axis quaternion.
        pose.pose.orientation.x = math.sin(joint5 / 2.0)
        pose.pose.orientation.w = math.cos(joint5 / 2.0)
        self.pose_pub.publish(pose)
        payload = {
            'state': 'STABLE', 'pixel_x': float(pixel[0]), 'pixel_y': float(pixel[1]),
            'robot_x': float(xy[0]), 'robot_y': float(xy[1]),
            'camera_x': float(raw_xy[0]), 'camera_y': float(raw_xy[1]),
            'correction_x_mm': float((raw_xy[0] - self.reference_camera_xy[0]) * 1000.0),
            'correction_y_mm': float((raw_xy[1] - self.reference_camera_xy[1]) * 1000.0),
            'joint5_rad': joint5, 'confidence': confidence,
            'z_source': 'coordinator_fixed_loading_place',
            'xy_spread_mm': xy_spread * 1000.0,
            'angle_spread_deg': math.degrees(angle_spread),
        }
        self.status_pub.publish(String(data=json.dumps(payload)))

    def on_image(self, message):
        try:
            source = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            annotated = source.copy()
            result = self.model.predict(
                source, conf=float(self.p('confidence_threshold')),
                device=str(self.p('device')), verbose=False)[0]
            selected = self.select_candidate(annotated, result)
            sample = None
            if selected is not None:
                _score, confidence, pixel, xy, joint5, _bounds, rectangle = selected
                sample = {'xy': xy, 'joint5': joint5}
                cv2.circle(annotated, tuple(np.round(pixel).astype(int)), 7, (0, 255, 255), -1)
                if rectangle is not None:
                    cv2.polylines(annotated, [rectangle.astype(int)], True, (255, 0, 255), 2)
            stable = self.stable_target(sample)
            if stable is not None:
                self.publish_target(stable, pixel, confidence, message.header)
                raw_xy, joint5, spread, _angle_spread = stable
                if self.taught_source_xy is None:
                    label = (f'TEACH REQUIRED camera X={raw_xy[0]:+.3f} '
                             f'Y={raw_xy[1]:+.3f} spread={spread*1000:.1f}mm')
                else:
                    xy = self.taught_source_xy + (raw_xy - self.reference_camera_xy)
                    label = (f'STABLE X={xy[0]:+.3f} Y={xy[1]:+.3f} '
                             f'q5={joint5:+.3f} spread={spread*1000:.1f}mm')
                cv2.putText(annotated, label,
                            (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.53, (0, 255, 255), 2)
            else:
                state = 'SEARCHING' if selected is None else \
                    f'STABILIZING {len(self.samples)}/{self.samples.maxlen}'
                cv2.putText(annotated, state, (8, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 180, 255), 2)
                self.status_pub.publish(String(data=json.dumps({'state': state})))
            annotated_message = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            annotated_message.header = message.header
            self.image_pub.publish(annotated_message)
        except Exception as error:  # noqa: BLE001 - camera/inference boundary
            self.samples.clear()
            self.status_pub.publish(String(data=json.dumps({
                'state': 'FAILED', 'reason': str(error)})))
            self.get_logger().error(f'unload vision inference failed: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = UnloadVisionTarget()
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
