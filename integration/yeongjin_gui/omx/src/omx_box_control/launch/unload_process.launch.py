#!/usr/bin/env python3
"""Standalone unloading OMX, camera, calibrated vision, and coordinator."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('omx_box_control')
    camera_launch = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'), 'launch',
        'camera_usb_cam.launch.py'])
    unload_launch = PathJoinSubstitution([
        share, 'launch', 'unload_omx_system.launch.py'])
    vision_config = PathJoinSubstitution([share, 'config', 'unload_marker.yaml'])

    return LaunchDescription([
        Node(package='rmw_zenoh_cpp', executable='rmw_zenohd',
             name='unload_zenohd', output='screen'),
        DeclareLaunchArgument(
            'port_name', description='Persistent /dev/serial/by-id path for unloading OMX'),
        DeclareLaunchArgument('video_device', default_value='/dev/video2'),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('start_coordinator', default_value='true'),
        DeclareLaunchArgument('teaching_mode', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(unload_launch),
            launch_arguments={
                'port_name': LaunchConfiguration('port_name'),
                'dry_run': LaunchConfiguration('dry_run'),
                'start_coordinator': LaunchConfiguration('start_coordinator'),
                'start_zenoh': 'false',
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            launch_arguments={
                'name': 'unload_camera',
                'video_device': LaunchConfiguration('video_device'),
            }.items()),
        TimerAction(
            period=3.0,
            actions=[Node(
                package='omx_box_control',
                executable='unload_marker_target_node.py',
                name='unload_marker_target',
                output='screen',
                parameters=[vision_config, {
                    'teaching_mode': ParameterValue(
                        LaunchConfiguration('teaching_mode'), value_type=bool)}])]),
    ])
