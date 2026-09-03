#!/usr/bin/env python3
"""Start camera/Yolo/operator nodes alongside the existing OMX pick flow."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('omx_box_control')
    config = PathJoinSubstitution([share, 'config', 'console.yaml'])
    homography = PathJoinSubstitution([share, 'config', 'homography_target.yaml'])
    pick_launch = PathJoinSubstitution([share, 'launch', 'pick_coordinator.launch.py'])
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(pick_launch)),
        Node(package='omx_box_control', executable='camera_homography_target_node.py',
             name='camera_homography_target', output='screen',
             parameters=[homography, {'show_window': False}]),
        Node(package='omx_box_control', executable='yolo_detection_node.py',
             name='yolo_detection', output='screen', parameters=[config]),
        Node(package='omx_box_control', executable='beagle_adapter_node.py',
             name='beagle_adapter', output='screen', parameters=[config, {
                 'connection_mode': ParameterValue(
                     EnvironmentVariable('BEAGLE_MODE', default_value='auto'),
                     value_type=str),
                 'trigger_host': ParameterValue(
                     EnvironmentVariable('BEAGLE_TRIGGER_HOST', default_value=''),
                     value_type=str),
                 'trigger_port': ParameterValue(
                     EnvironmentVariable('BEAGLE_TRIGGER_PORT', default_value='8765'),
                     value_type=int),
                 'status_port': ParameterValue(
                     EnvironmentVariable('BEAGLE_STATUS_PORT', default_value='9000'),
                     value_type=int),
             }]),
        Node(package='omx_box_control', executable='sorting_orchestrator_node.py',
             name='sorting_orchestrator', output='screen', parameters=[config]),
        Node(package='omx_box_control', executable='operations_log_node.py',
             name='operations_log', output='screen', parameters=[config]),
        Node(package='omx_box_control', executable='omx_console.py',
             name='omx_console', output='screen'),
    ])
