#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('omx_box_control'), 'config', 'rule_based_pick.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config),
        Node(package='omx_box_control', executable='rule_based_pick_node.py', name='rule_based_pick',
             output='screen', parameters=[LaunchConfiguration('config_file')]),
    ])
