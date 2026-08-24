#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('omx_box_control'), 'config', 'homography_target.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        Node(package='omx_box_control',
             executable='camera_homography_target_node.py',
             name='camera_homography_target', output='screen',
             parameters=[LaunchConfiguration('config_file')]),
    ])
