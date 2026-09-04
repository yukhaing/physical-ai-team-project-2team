#!/usr/bin/env python3
"""Bring up a second OMX and its automatic unloading coordinator."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    port_name = LaunchConfiguration('port_name')
    controller_config = LaunchConfiguration('controller_config')
    unload_config = LaunchConfiguration('unload_config')
    init_position = LaunchConfiguration('init_position')
    dry_run = LaunchConfiguration('dry_run')
    start_coordinator = LaunchConfiguration('start_coordinator')
    start_zenoh = LaunchConfiguration('start_zenoh')
    bringup = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'), 'launch', 'omx_f.launch.py'])
    movej = PathJoinSubstitution([
        FindPackageShare('cyclo_motion_controller_ros'), 'launch',
        'omx_controller.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='unload_omx'),
        # Keep delayed conditions from the included ROBOTIS launch in scope.
        DeclareLaunchArgument('start_rviz', default_value='false'),
        # Do not move a newly connected unloading arm merely by launching it.
        # The unloading coordinator performs the first feedback-gated staging move.
        DeclareLaunchArgument('init_position', default_value='false'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('start_coordinator', default_value='true'),
        DeclareLaunchArgument('start_zenoh', default_value='true'),
        Node(package='rmw_zenoh_cpp', executable='rmw_zenohd',
             name='unload_zenohd', output='screen',
             condition=IfCondition(start_zenoh)),
        # The vendor launch evaluates these arguments in delayed actions. Declare
        # them in the parent scope so they remain available after inclusion.
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument('use_sim', default_value='false'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),
        DeclareLaunchArgument('mock_sensor_commands', default_value='false'),
        DeclareLaunchArgument('ros2_control_type', default_value='omx_f'),
        DeclareLaunchArgument(
            'init_position_file', default_value='initial_positions.yaml'),
        DeclareLaunchArgument(
            'port_name',
            description='Persistent /dev/serial/by-id path for the unloading OMX'),
        DeclareLaunchArgument(
            'controller_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('omx_box_control'), 'config',
                'unload_omx_controller.yaml'])),
        DeclareLaunchArgument(
            'unload_config', default_value=PathJoinSubstitution([
                FindPackageShare('omx_box_control'), 'config',
                'unload_coordinator.yaml'])),
        GroupAction([
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bringup),
                launch_arguments={
                    'start_rviz': 'false', 'port_name': port_name,
                    'init_position': init_position}.items()),
            TimerAction(
                period=8.0,
                actions=[IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(movej),
                    launch_arguments={
                        'controller_type': 'movej',
                        'start_interactive_marker': 'false',
                        'config_file': controller_config}.items())]),
            TimerAction(
                period=12.0,
                actions=[Node(
                    package='omx_box_control',
                    executable='unload_coordinator_node.py',
                    name='unload_coordinator', output='screen',
                    condition=IfCondition(start_coordinator),
                    parameters=[unload_config, {
                        'dry_run': ParameterValue(dry_run, value_type=bool)}])]),
        ]),
    ])
