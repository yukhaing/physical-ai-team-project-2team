#!/usr/bin/env python3
"""Keyboard Cartesian jogging and explicit place-point teaching for OMX-F."""

import math
from pathlib import Path
import re
import select
import sys
import termios
import tty

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from robotis_interfaces.msg import MoveL
from sensor_msgs.msg import JointState


HELP = """
Place keyboard teach
  W/S : X +/-       A/D : Y +/-       R/F : Z +/-
  1   : 1 mm step   5   : 5 mm step   0   : 10 mm step
  P   : print actual TCP coordinates
  V   : save actual TCP as remembered_place_xyz
  H   : show this help       X or ESC : exit
Wait for each small move to settle before pressing the next motion key.
"""


class PlaceKeyboardTeach(Node):
    def __init__(self):
        super().__init__('place_keyboard_teach')
        defaults = {
            'current_pose_topic': '/omx_movel_controller/current_pose',
            'movel_topic': '/omx_movel_controller/movel',
            'joint_states_topic': '/joint_states',
            'base_frame': 'link0',
            'move_duration': 0.3,
            'step': 0.005,
            'min_x': 0.08,
            'max_x': 0.32,
            'max_abs_y': 0.25,
            'min_z': 0.03,
            'max_z': 0.25,
            'save_min_z': 0.10,
            'save_max_z': 0.18,
            'fk_position_bias': [0.00126, 0.0, 0.00055],
            'config_path': (
                '/root/omx_box_project_ws/src/omx_box_control/'
                'config/pick_coordinator.yaml'),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.pose = None
        self.positions = {}
        self.step = float(self.get_parameter('step').value)
        self.publisher = self.create_publisher(
            MoveL, self.get_parameter('movel_topic').value, 10)
        self.create_subscription(
            PoseStamped, self.get_parameter('current_pose_topic').value,
            self.on_pose, 10)
        self.create_subscription(
            JointState, self.get_parameter('joint_states_topic').value,
            self.on_joints, 10)

    def p(self, name):
        return self.get_parameter(name).value

    def on_pose(self, message):
        self.pose = message

    def on_joints(self, message):
        self.positions.update(zip(message.name, message.position))

    @staticmethod
    def translation(x, y, z):
        matrix = np.eye(4)
        matrix[:3, 3] = [x, y, z]
        return matrix

    @staticmethod
    def rotation(axis, angle):
        cosine, sine = math.cos(angle), math.sin(angle)
        matrix = np.eye(4)
        if axis == 'z':
            matrix[:3, :3] = [
                [cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]]
        elif axis == 'y':
            matrix[:3, :3] = [
                [cosine, 0, sine], [0, 1, 0], [-sine, 0, cosine]]
        else:
            matrix[:3, :3] = [
                [1, 0, 0], [0, cosine, -sine], [0, sine, cosine]]
        return matrix

    def actual_transform(self):
        names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        if not all(name in self.positions for name in names):
            return None
        q1, q2, q3, q4, q5 = [self.positions[name] for name in names]
        matrix = (
            self.translation(-0.01125, 0.0, 0.034) @ self.rotation('z', q1) @
            self.translation(0.0, 0.0, 0.0635) @ self.rotation('y', q2) @
            self.translation(0.0415, 0.0, 0.11315) @ self.rotation('y', q3) @
            self.translation(0.162, 0.0, 0.0) @ self.rotation('y', q4) @
            self.translation(0.0287, 0.0, 0.0) @ self.rotation('x', q5) @
            self.translation(0.09193, -0.0016, 0.0))
        matrix[:3, 3] += np.asarray(self.p('fk_position_bias'), dtype=float)
        return matrix

    def actual_xyz(self):
        matrix = self.actual_transform()
        return None if matrix is None else matrix[:3, 3].copy()

    @staticmethod
    def matrix_quaternion(matrix):
        """Return ROS x/y/z/w quaternion for a proper rotation matrix."""
        trace = float(np.trace(matrix))
        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            w = 0.25 * scale
            x = (matrix[2, 1] - matrix[1, 2]) / scale
            y = (matrix[0, 2] - matrix[2, 0]) / scale
            z = (matrix[1, 0] - matrix[0, 1]) / scale
        else:
            index = int(np.argmax(np.diag(matrix)))
            if index == 0:
                scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
                w = (matrix[2, 1] - matrix[1, 2]) / scale
                x = 0.25 * scale
                y = (matrix[0, 1] + matrix[1, 0]) / scale
                z = (matrix[0, 2] + matrix[2, 0]) / scale
            elif index == 1:
                scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
                w = (matrix[0, 2] - matrix[2, 0]) / scale
                x = (matrix[0, 1] + matrix[1, 0]) / scale
                y = 0.25 * scale
                z = (matrix[1, 2] + matrix[2, 1]) / scale
            else:
                scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
                w = (matrix[1, 0] - matrix[0, 1]) / scale
                x = (matrix[0, 2] + matrix[2, 0]) / scale
                y = (matrix[1, 2] + matrix[2, 1]) / scale
                z = 0.25 * scale
        return x, y, z, w

    def safe(self, x, y, z):
        return (
            float(self.p('min_x')) <= x <= float(self.p('max_x')) and
            abs(y) <= float(self.p('max_abs_y')) and
            float(self.p('min_z')) <= z <= float(self.p('max_z')))

    def jog(self, dx, dy, dz):
        actual = self.actual_transform()
        if actual is None:
            print('Actual joint feedback is unavailable; check bringup.')
            return
        source_xyz = actual[:3, 3]
        x, y, z = (
            source_xyz[0] + dx,
            source_xyz[1] + dy,
            source_xyz[2] + dz)
        if not self.safe(x, y, z):
            print(f'BLOCKED outside workspace: x={x:.4f}, y={y:.4f}, z={z:.4f}')
            return
        command = MoveL()
        command.pose.header.stamp = self.get_clock().now().to_msg()
        command.pose.header.frame_id = self.p('base_frame')
        command.pose.pose.position.x = x
        command.pose.pose.position.y = y
        command.pose.pose.position.z = z
        qx, qy, qz, qw = self.matrix_quaternion(actual[:3, :3])
        command.pose.pose.orientation.x = qx
        command.pose.pose.orientation.y = qy
        command.pose.pose.orientation.z = qz
        command.pose.pose.orientation.w = qw
        duration = max(0.2, float(self.p('move_duration')))
        command.time_from_start.sec = int(duration)
        command.time_from_start.nanosec = int((duration % 1.0) * 1.0e9)
        self.publisher.publish(command)
        print(f'MOVE target x={x:.4f}, y={y:.4f}, z={z:.4f}, step={self.step*1000:.0f}mm')

    def print_actual(self):
        xyz = self.actual_xyz()
        if xyz is None:
            print('Actual joint feedback is unavailable.')
            return None
        print(f'ACTUAL TCP x={xyz[0]:.5f}, y={xyz[1]:.5f}, z={xyz[2]:.5f} m')
        return xyz

    def save(self):
        xyz = self.print_actual()
        if xyz is None:
            return
        if not (float(self.p('save_min_z')) <= xyz[2] <= float(self.p('save_max_z'))):
            print(
                f'BLOCKED save: Z must be {float(self.p("save_min_z"))*1000:.0f}-'
                f'{float(self.p("save_max_z"))*1000:.0f}mm for a place approach point.')
            return
        path = Path(str(self.p('config_path')))
        try:
            original = path.read_text(encoding='utf-8')
        except OSError as error:
            print(f'Cannot read config: {error}')
            return
        value = f'remembered_place_xyz: [{xyz[0]:.5f}, {xyz[1]:.5f}, {xyz[2]:.5f}]'
        updated, count = re.subn(
            r'remembered_place_xyz:\s*\[[^\]]+\]', value, original, count=1)
        if count != 1:
            print('Cannot find exactly one remembered_place_xyz entry; nothing saved.')
            return
        temporary = path.with_suffix(path.suffix + '.tmp')
        try:
            temporary.write_text(updated, encoding='utf-8')
            temporary.replace(path)
        except OSError as error:
            print(f'Cannot save config: {error}')
            return
        print(f'SAVED {value}')
        print('Restart pick_coordinator.launch.py before using the new point.')


def main(args=None):
    if not sys.stdin.isatty():
        raise RuntimeError('place keyboard teach requires an interactive terminal')
    rclpy.init(args=args)
    node = PlaceKeyboardTeach()
    old_settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            readable, _, _ = select.select([sys.stdin], [], [], 0.02)
            if not readable:
                continue
            key = sys.stdin.read(1).lower()
            if key in ('x', '\x1b'):
                break
            if key == 'h':
                print(HELP)
            elif key == '1':
                node.step = 0.001
                print('Step = 1mm')
            elif key == '5':
                node.step = 0.005
                print('Step = 5mm')
            elif key == '0':
                node.step = 0.010
                print('Step = 10mm')
            elif key == 'p':
                node.print_actual()
            elif key == 'v':
                node.save()
            elif key in 'wsadrf':
                delta = {
                    'w': (node.step, 0.0, 0.0),
                    's': (-node.step, 0.0, 0.0),
                    'a': (0.0, node.step, 0.0),
                    'd': (0.0, -node.step, 0.0),
                    'r': (0.0, 0.0, node.step),
                    'f': (0.0, 0.0, -node.step),
                }[key]
                node.jog(*delta)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
