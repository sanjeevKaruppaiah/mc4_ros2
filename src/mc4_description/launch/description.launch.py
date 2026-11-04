"""Publish the MC 4.0 URDF on /robot_description and its static TFs.

Included by mc4_bringup/robot.launch.py. Dimensions are launch args so you can
match your physical build without editing the xacro.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

DIMENSIONS = {
    'four_wheel': 'true',
    'wheel_radius': '0.0325',
    'wheel_separation': '0.250',
    'wheelbase': '0.300',
    'lidar_x': '0.0',
    'lidar_z': '0.180',
    'lidar_yaw': '0.0',
}


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('mc4_description'), 'urdf', 'mc4.urdf.xacro'])

    args = [DeclareLaunchArgument('use_sim_time', default_value='false')]
    xacro_cmd = ['xacro ', xacro_file]
    for name, default in DIMENSIONS.items():
        args.append(DeclareLaunchArgument(name, default_value=default))
        xacro_cmd += [f' {name}:=', LaunchConfiguration(name)]

    robot_description = ParameterValue(Command(xacro_cmd), value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription(args + [rsp])
