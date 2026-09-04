#!/usr/bin/env python3
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('omx_box_control'), 'config', 'movej_smooth_relay.yaml'])
    return LaunchDescription([
        Node(package='omx_box_control', executable='movej_smooth_relay_node.py',
             name='movej_smooth_relay', output='screen',
             parameters=[config]),
    ])
