#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('omx_box_control'), 'config', 'homography_7point_calibration.yaml'])
    return LaunchDescription([Node(
        package='omx_box_control',
        executable='camera_homography_7point_calibration_node.py',
        name='camera_homography_7point_calibration',
        output='screen',
        parameters=[config],
    )])
