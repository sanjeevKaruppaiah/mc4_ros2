"""Simulated 360 deg 2D lidar for fake-hardware mode.

Reads the robot pose from /odom (which is ground truth in fake mode),
ray-casts against mc4_base.world, and publishes sensor_msgs/LaserScan on
/scan in laser_frame, so slam_toolbox and Nav2 behave as with a real
RPLIDAR / LD19.
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from mc4_base.world import default_world, raycast


class FakeLidarNode(Node):

    def __init__(self):
        super().__init__('fake_lidar_node')
        self.frame_id = self.declare_parameter('frame_id', 'laser_frame').value
        self.rate = float(self.declare_parameter('rate', 10.0).value)
        self.samples = int(self.declare_parameter('samples', 360).value)
        self.range_min = float(self.declare_parameter('range_min', 0.12).value)
        self.range_max = float(self.declare_parameter('range_max', 8.0).value)
        self.noise = float(self.declare_parameter('noise_std', 0.01).value)
        # Lidar mount offset from base_footprint (must match the URDF laser_joint).
        self.lidar_x = float(self.declare_parameter('lidar_x', 0.0).value)
        self.lidar_yaw = float(self.declare_parameter('lidar_yaw', 0.0).value)

        self.world = default_world()
        self.pose = None
        self.rng = np.random.default_rng()
        inc = 2.0 * math.pi / self.samples
        self.beam_angles = -math.pi + inc * np.arange(self.samples)

        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.create_timer(1.0 / self.rate, self._publish)
        self.get_logger().info('Fake lidar running in a 6x4 m virtual arena')

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _publish(self):
        x, y, yaw = self.pose if self.pose is not None else (0.0, 0.0, 0.0)
        lx = x + self.lidar_x * math.cos(yaw)
        ly = y + self.lidar_x * math.sin(yaw)
        laser_yaw = yaw + self.lidar_yaw

        ranges = raycast(self.world, lx, ly, laser_yaw + self.beam_angles, self.range_max)
        finite = np.isfinite(ranges)
        ranges[finite] += self.rng.normal(0.0, self.noise, int(finite.sum()))
        ranges[finite & (ranges < self.range_min)] = np.inf

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = float(self.beam_angles[0])
        scan.angle_max = float(self.beam_angles[-1])
        scan.angle_increment = 2.0 * math.pi / self.samples
        scan.scan_time = 1.0 / self.rate
        scan.time_increment = scan.scan_time / self.samples
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.astype(np.float32).tolist()
        self.pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = FakeLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
