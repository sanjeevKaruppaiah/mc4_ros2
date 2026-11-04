"""One-command demo with NO hardware: fake base + fake lidar + SLAM + Nav2 + RViz.

    ros2 launch mc4_bringup sim.launch.py

Click "2D Goal Pose" in RViz and watch the robot plan and drive while it
maps the virtual 6 x 4 m arena. Pass rviz:=false on a headless machine.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('mc4_bringup')

    def include(name, condition=None, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, 'launch', name])),
            launch_arguments=kwargs.items(), condition=condition)

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        # Fake robot has perfect wheel odometry (the fake lidar uses it as ground truth).
        include('robot.launch.py', use_fake_hardware='true', lidar='fake', odom_source='wheel'),
        # Give TF and /scan a moment before SLAM and Nav2 start.
        TimerAction(period=3.0, actions=[include('navigation.launch.py', slam='true')]),
        include('rviz.launch.py', condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
