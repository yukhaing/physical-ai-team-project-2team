#!/usr/bin/env python3
"""Route two physical camera feeds to stable loading/unloading role topics."""

import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml


class CameraRoleRouter(Node):
    def __init__(self):
        super().__init__('camera_role_router')
        defaults = {
            'device_a_topic': '/camera_devices/loading/image_raw',
            'device_b_topic': '/camera_devices/unloading/image_raw',
            'loading_role_topic': '/camera/image_raw',
            'unloading_role_topic': '/unload_camera/image_raw',
            'status_topic': '/camera_roles/status',
            'device_a_path': '/dev/video0',
            'device_b_path': '/dev/video2',
            'state_file': '/root/omx_box_project_ws/integration/yeongjin_gui/runtime/camera_roles.yaml',
            'camera_timeout': 3.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.swapped = False
        self.last_seen = {'a': None, 'b': None}
        self._load_state()
        self.loading_pub = self.create_publisher(
            Image, self.p('loading_role_topic'), qos_profile_sensor_data)
        self.unloading_pub = self.create_publisher(
            Image, self.p('unloading_role_topic'), qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, self.p('status_topic'), 10)
        self.create_subscription(
            Image, self.p('device_a_topic'), lambda message: self.route('a', message),
            qos_profile_sensor_data)
        self.create_subscription(
            Image, self.p('device_b_topic'), lambda message: self.route('b', message),
            qos_profile_sensor_data)
        self.create_service(Trigger, '/camera_roles/swap', self.swap)
        self.create_timer(1.0, self.publish_status)
        self.publish_status()

    def p(self, name):
        return str(self.get_parameter(name).value)

    def _load_state(self):
        path = Path(self.p('state_file'))
        if not path.is_file():
            return
        try:
            data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
            self.swapped = bool(data.get('swapped', False))
        except (OSError, yaml.YAMLError, TypeError) as error:
            self.get_logger().warning(f'Ignoring invalid camera role state: {error}')

    def _save_state(self):
        path = Path(self.p('state_file'))
        path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            'swapped': self.swapped,
            'device_a': self.p('device_a_path'),
            'device_b': self.p('device_b_path'),
        }
        temporary = path.with_suffix(path.suffix + '.tmp')
        temporary.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
        os.replace(temporary, path)

    def route(self, source, message):
        self.last_seen[source] = time.monotonic()
        is_loading = (source == 'b') if self.swapped else (source == 'a')
        (self.loading_pub if is_loading else self.unloading_pub).publish(message)

    def missing(self):
        now = time.monotonic()
        timeout = float(self.get_parameter('camera_timeout').value)
        return [self.p(f'device_{source}_path') for source in ('a', 'b')
                if self.last_seen[source] is None or now - self.last_seen[source] > timeout]

    def swap(self, _request, response):
        missing = self.missing()
        if missing:
            response.success = False
            response.message = 'camera role swap blocked; missing: ' + ', '.join(missing)
            self.get_logger().error(response.message)
            self.publish_status()
            return response
        previous = self.swapped
        self.swapped = not self.swapped
        try:
            self._save_state()
        except OSError as error:
            self.swapped = previous
            response.success = False
            response.message = f'camera role state was not saved: {error}'
            self.get_logger().error(response.message)
            return response
        response.success = True
        response.message = 'camera roles swapped' if self.swapped else 'camera roles restored'
        self.get_logger().info(response.message)
        self.publish_status()
        return response

    def publish_status(self):
        missing = self.missing()
        payload = {
            'swapped': self.swapped,
            'loading_device': self.p('device_b_path' if self.swapped else 'device_a_path'),
            'unloading_device': self.p('device_a_path' if self.swapped else 'device_b_path'),
            'missing_devices': missing,
            'ready': not missing,
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = CameraRoleRouter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
