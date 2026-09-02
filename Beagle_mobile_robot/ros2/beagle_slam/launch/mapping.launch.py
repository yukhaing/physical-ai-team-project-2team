from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('beagle_slam')
    base_params = PathJoinSubstitution([package_share, 'config', 'beagle_base.yaml'])
    slam_params = PathJoinSubstitution([package_share, 'config', 'slam_toolbox.yaml'])
    slam_launch = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])

    port_name = LaunchConfiguration('port_name')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('port_name', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        Node(
            package='beagle_slam',
            executable='beagle_base_node.py',
            name='beagle_base',
            output='screen',
            parameters=[base_params, {'port_name': port_name}],
            on_exit=Shutdown(reason='Beagle base exited'),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'slam_params_file': slam_params,
                'use_sim_time': 'false',
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='beagle_mapping_rviz',
            arguments=['-d', PathJoinSubstitution([package_share, 'config', 'mapping.rviz'])],
            output='screen',
            condition=IfCondition(use_rviz),
        ),
    ])
