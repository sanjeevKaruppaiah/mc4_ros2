"""Bring up the MC 4.0 robot: URDF/TF + base driver + lidar + odometry.

Real robot (ESP32, no encoders, RPLIDAR A1):
    ros2 launch mc4_bringup robot.launch.py \
        base_port:=/dev/mc4_base lidar_port:=/dev/mc4_lidar
No hardware:
    ros2 launch mc4_bringup robot.launch.py use_fake_hardware:=true lidar:=fake odom_source:=wheel

odom_source decides who publishes /odom and the odom -> base_footprint TF:
    lidar  rf2o_laser_odometry estimates motion from consecutive scans.
           Use this when the robot has NO wheel encoders (current MC 4.0 build).
           The base driver's dead-reckoning goes to /wheel_odom (no TF).
    wheel  mc4_base_node publishes /odom + TF (encoders, or the fake robot).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('mc4_bringup')
    lc = LaunchConfiguration
    lidar_odom = PythonExpression(["'", lc('odom_source'), "' == 'lidar'"])

    args = [
        DeclareLaunchArgument('use_fake_hardware', default_value='false',
                              description='Simulate the base instead of opening a serial port'),
        DeclareLaunchArgument('use_encoders', default_value='false',
                              description='true if the motors have encoders and HAS_ENCODERS=1 in firmware'),
        DeclareLaunchArgument('odom_source', default_value='lidar',
                              description='lidar (rf2o, no encoders) | wheel (encoders or fake)'),
        DeclareLaunchArgument('base_port', default_value='/dev/ttyUSB0',
                              description='Serial port of the MC 4.0 ESP32'),
        DeclareLaunchArgument('lidar', default_value='rplidar_a1',
                              description='rplidar_a1 | rplidar_c1 | ldlidar_ld19 | ydlidar_x4 | fake | none'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1'),
        # Physical dimensions shared by URDF and kinematics (MC 4.0 build sheet).
        DeclareLaunchArgument('wheel_radius', default_value='0.0325',
                              description='65 mm wheels'),
        DeclareLaunchArgument('wheel_separation', default_value='0.250',
                              description='Track width (left-right wheel centres). Tune for skid steer.'),
        DeclareLaunchArgument('wheelbase', default_value='0.300',
                              description='Front-rear axle distance (4-wheel chassis)'),
        DeclareLaunchArgument('four_wheel', default_value='true',
                              description='true = 4WD skid steer, false = 2 wheels + caster'),
        DeclareLaunchArgument('lidar_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.180',
                              description='Lidar scan plane height above the floor'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0',
                              description='Lidar mount yaw (rad); 3.14159 if scans appear behind the robot'),
    ]

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('mc4_description'), 'launch', 'description.launch.py'])),
        launch_arguments={
            'wheel_radius': lc('wheel_radius'),
            'wheel_separation': lc('wheel_separation'),
            'wheelbase': lc('wheelbase'),
            'four_wheel': lc('four_wheel'),
            'lidar_x': lc('lidar_x'),
            'lidar_z': lc('lidar_z'),
            'lidar_yaw': lc('lidar_yaw'),
        }.items())

    base = Node(
        package='mc4_base',
        executable='mc4_base_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([bringup, 'config', 'base.yaml']),
            {
                'use_fake_hardware': lc('use_fake_hardware'),
                'use_encoders': lc('use_encoders'),
                'port': lc('base_port'),
                'wheel_radius': lc('wheel_radius'),
                'wheel_separation': lc('wheel_separation'),
                # With lidar odometry, the drifting dead-reckoning must not own /odom or TF.
                'odom_topic': PythonExpression(
                    ["'wheel_odom' if '", lc('odom_source'), "' == 'lidar' else 'odom'"]),
                'publish_tf': PythonExpression(["'", lc('odom_source'), "' != 'lidar'"]),
            },
        ],
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, 'launch', 'lidar.launch.py'])),
        launch_arguments={
            'lidar': lc('lidar'),
            'lidar_port': lc('lidar_port'),
            'lidar_x': lc('lidar_x'),
            'lidar_yaw': lc('lidar_yaw'),
        }.items())

    # Laser scan-matching odometry (source build, see deps.repos).
    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        condition=IfCondition(lidar_odom),
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0,
        }])

    return LaunchDescription(args + [description, base, lidar, rf2o])
