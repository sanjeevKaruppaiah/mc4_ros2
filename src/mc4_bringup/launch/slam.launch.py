"""Online SLAM with slam_toolbox (builds /map and the map -> odom TF).

    ros2 launch mc4_bringup slam.launch.py
Save the map when done:
    ros2 run nav2_map_server map_saver_cli -f ~/maps/arena

Uses slam_toolbox's own launch file so lifecycle handling stays correct
across ROS 2 releases.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare('mc4_bringup'), 'config', 'slam_toolbox.yaml'])

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
        }.items())

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('slam_params_file', default_value=default_params),
        slam,
    ])
