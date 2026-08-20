#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def configured_node(executable, name, config, extra=None):
    parameters = [config]
    if extra:
        parameters.append(extra)
    return Node(package='omx_box_control', executable=executable,
                name=name, output='screen', parameters=parameters)


def generate_launch_description():
    share = FindPackageShare('omx_box_control')
    config = lambda name: PathJoinSubstitution([share, 'config', name])
    real = {'dry_run': ParameterValue(False, value_type=bool)}
    grasped = dict(
        real, min_grasp_position=ParameterValue(0.005, value_type=float))
    # A single selected pick target is intentionally retained for the full pick
    # side of the cycle. The coordinator itself requires it to be fresh at start.
    retained_target = dict(real, target_max_age=ParameterValue(300.0, value_type=float))
    return LaunchDescription([
        DeclareLaunchArgument(
            'coordinator_config', default_value=config('pick_coordinator.yaml')),
        configured_node('pick_coordinator_node.py', 'pick_coordinator',
                        LaunchConfiguration('coordinator_config')),
        configured_node('movej_staging_node.py', 'movej_staging',
                        config('movej_staging.yaml'), dict(
                            real, movej_topic='/pick_coordinator/commands/staging')),
        configured_node('movej_xy_approach_node.py', 'movej_xy_approach',
                        config('movej_xy_approach.yaml'), dict(
                            retained_target,
                            movej_topic='/pick_coordinator/commands/xy')),
        configured_node('movej_xy_approach_node.py', 'movej_pitch_pregrasp',
                        config('movej_pitch_pregrasp.yaml'), dict(
                            retained_target,
                            movej_topic='/pick_coordinator/commands/pitch')),
        configured_node('gripper_open_node.py', 'gripper_open',
                        config('gripper_open.yaml'), retained_target),
        configured_node('movej_single_ptp_descent_node.py',
                        'movej_single_ptp_descent',
                        config('movej_single_ptp_descent.yaml'), dict(
                            retained_target,
                            movej_topic='/pick_coordinator/commands/pick_descent')),
        configured_node('movej_loaded_lift_node.py', 'movej_lift',
                        config('movej_lift.yaml'), dict(
                            grasped, movej_topic='/pick_coordinator/commands/lift')),
        configured_node('movej_xy_approach_node.py', 'movej_place_xy_transfer',
                        config('movej_place_xy_transfer.yaml'), dict(
                            real,
                            target_topic='/pick_coordinator/place_target',
                            movej_topic='/pick_coordinator/commands/place_xy_transfer')),
        configured_node('movej_xy_approach_node.py', 'movej_place_recovery',
                        config('movej_place_recovery.yaml'), dict(
                            real,
                            target_topic='/pick_coordinator/place_target',
                            movej_topic='/pick_coordinator/commands/place_recovery')),
        configured_node('movej_single_ptp_descent_node.py',
                        'movej_single_ptp_place_descent',
                        config('movej_single_ptp_place_descent.yaml'), dict(
                            retained_target,
                            target_topic='/pick_coordinator/place_release_target',
                            movej_topic='/pick_coordinator/commands/place_descent')),
    ])
