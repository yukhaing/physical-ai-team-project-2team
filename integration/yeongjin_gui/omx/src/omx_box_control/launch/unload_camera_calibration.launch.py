#!/usr/bin/env python3
"""Run the unloading camera and its independent seven-point calibration UI."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('omx_box_control')
    camera_launch = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'), 'launch',
        'camera_usb_cam.launch.py'])
    calibration_config = PathJoinSubstitution([
        share, 'config', 'unload_homography_7point_calibration.yaml'])
    return LaunchDescription([
        Node(package='rmw_zenoh_cpp', executable='rmw_zenohd',
             name='unload_calibration_zenohd', output='screen'),
        DeclareLaunchArgument('video_device', default_value='/dev/video2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            launch_arguments={
                'name': 'unload_camera',
                'video_device': LaunchConfiguration('video_device'),
            }.items()),
        TimerAction(period=2.0, actions=[Node(
            package='omx_box_control',
            executable='camera_homography_7point_calibration_node.py',
            name='camera_homography_7point_calibration',
            output='screen',
            parameters=[calibration_config])]),
    ])

