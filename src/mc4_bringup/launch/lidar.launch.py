"""Start one 2D lidar driver, chosen by the `lidar` argument.

Every driver publishes sensor_msgs/LaserScan on /scan with frame_id
laser_frame, so SLAM and Nav2 never need to know which lidar is fitted.

    lidar:=rplidar_a1    Slamtec RPLIDAR A1/A2  (apt: ros-jazzy-rplidar-ros)
    lidar:=rplidar_c1    Slamtec RPLIDAR C1     (apt: ros-jazzy-rplidar-ros)
    lidar:=ldlidar_ld19  LDROBOT LD19 / LD06    (source: deps.repos)
    lidar:=ydlidar_x4    YDLIDAR X4 / X2        (source + YDLidar-SDK)
    lidar:=fake          simulated arena, pairs with use_fake_hardware:=true
    lidar:=none          nothing (e.g. when replaying a rosbag)

To add another lidar: add a Node below with the same /scan topic and
laser_frame frame_id, guarded by its own `lidar:=<name>` condition.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

FRAME = 'laser_frame'


def when(name):
    return IfCondition(PythonExpression(["'", LaunchConfiguration('lidar'), "' == '", name, "'"]))


def generate_launch_description():
    port = LaunchConfiguration('lidar_port')

    # Default MC 4.0 lidar. A1M8: 115200 baud, ~5.5 Hz, 0.15-12 m.
    # If the map comes out mirrored/rotated, fix it with robot.launch.py lidar_yaw:=
    # (see docs/ESP32_RPLIDAR_A1.md "Check lidar orientation").
    rplidar_a1 = Node(
        package='rplidar_ros', executable='rplidar_node', name='rplidar_node',
        output='screen', condition=when('rplidar_a1'),
        parameters=[{
            'serial_port': port,
            'serial_baudrate': 115200,
            'frame_id': FRAME,
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }])

    rplidar_c1 = Node(
        package='rplidar_ros', executable='rplidar_node', name='rplidar_node',
        output='screen', condition=when('rplidar_c1'),
        parameters=[{
            'serial_port': port,
            'serial_baudrate': 460800,
            'frame_id': FRAME,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }])

    ld19 = Node(
        package='ldlidar_stl_ros2', executable='ldlidar_stl_ros2_node', name='ldlidar',
        output='screen', condition=when('ldlidar_ld19'),
        parameters=[{
            'product_name': 'LDLiDAR_LD19',
            'topic_name': 'scan',
            'frame_id': FRAME,
            'port_name': port,
            'port_baudrate': 230400,
            'laser_scan_dir': True,
            'enable_angle_crop_func': False,
        }])

    ydlidar = Node(
        package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node', name='ydlidar',
        output='screen', condition=when('ydlidar_x4'),
        parameters=[{
            'port': port,
            'frame_id': FRAME,
            'baudrate': 128000,
            'lidar_type': 1,
            'device_type': 0,
            'sample_rate': 5,
            'isSingleChannel': False,
            'support_motor_dtr': True,
            'reversion': False,
            'inverted': True,
            'auto_reconnect': True,
            'angle_max': 180.0,
            'angle_min': -180.0,
            'range_max': 10.0,
            'range_min': 0.12,
            'frequency': 8.0,
        }])

    fake = Node(
        package='mc4_base', executable='fake_lidar_node', name='fake_lidar_node',
        output='screen', condition=when('fake'),
        parameters=[{
            'frame_id': FRAME,
            'lidar_x': LaunchConfiguration('lidar_x'),
            'lidar_yaw': LaunchConfiguration('lidar_yaw'),
        }])

    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='rplidar_a1'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('lidar_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),
        rplidar_a1, rplidar_c1, ld19, ydlidar, fake,
    ])
