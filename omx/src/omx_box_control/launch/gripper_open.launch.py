#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('omx_box_control'), 'config', 'gripper_open.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config),
        Node(package='omx_box_control', executable='gripper_open_node.py',
             name='gripper_open', output='screen',
             parameters=[LaunchConfiguration('config_file')]),
    ])
