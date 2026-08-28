#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('omx_box_control'), 'config', 'yolo_target_bridge.yaml'])
    return LaunchDescription([
        Node(
            package='omx_box_control',
            executable='yolo_target_bridge_node.py',
            name='yolo_target_bridge',
            output='screen',
            parameters=[config],
        ),
    ])
