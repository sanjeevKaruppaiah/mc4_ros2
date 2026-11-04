"""RViz with the MC 4.0 layout: robot, TF, lidar, map, costmaps, paths,
and the 2D Pose Estimate / 2D Goal Pose tools.

    ros2 launch mc4_bringup rviz.launch.py
Run this on a laptop on the same network/ROS_DOMAIN_ID as the robot.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_cfg = PathJoinSubstitution([FindPackageShare('mc4_bringup'), 'rviz', 'mc4.rviz'])
    return LaunchDescription([
        DeclareLaunchArgument('rviz_config', default_value=default_cfg),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', LaunchConfiguration('rviz_config')],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
