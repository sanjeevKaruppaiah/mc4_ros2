"""MC 4.0 base driver.

Subscribes : /cmd_vel        geometry_msgs/Twist
Publishes  : /odom           nav_msgs/Odometry
             /joint_states   sensor_msgs/JointState  (wheel joints, for RViz)
             TF odom -> base_footprint (if publish_tf)

Talks to the MC 4.0 microcontroller with the line protocol in docs/PROTOCOL.md:
    host -> MCU :  "V <left_rad_s> <right_rad_s>\n"
    MCU  -> host:  "E <left_ticks> <right_ticks>\n"   (cumulative encoder counts)

Odometry sources:
  use_encoders:=true   E lines from the MCU (closed loop, best).
  use_encoders:=false  NO encoders: commanded wheel speeds are integrated
                       ("dead reckoning"). This drifts, so on the real robot
                       it is published on /wheel_odom without TF and the lidar
                       odometry (rf2o) owns /odom + TF. See robot.launch.py.
  use_fake_hardware    no serial port; commanded speeds give perfect ticks, so
                       the whole stack (SLAM, Nav2, RViz) runs on a laptop.
"""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from mc4_base.kinematics import DiffDriveOdometry, scale_to_limit, wheel_speeds_from_twist

try:
    import serial  # pyserial
except ImportError:  # only needed for real hardware
    serial = None


class Mc4BaseNode(Node):

    def __init__(self):
        super().__init__('mc4_base_node')

        # dynamic_typing lets YAML say 1320 or 1320.0 without a type error.
        dyn = ParameterDescriptor(dynamic_typing=True)

        def p(name, default):
            return self.declare_parameter(name, default, dyn)

        def num(name, default):
            return float(p(name, default).value)

        def flag(name, default):
            v = p(name, default).value
            return v.strip().lower() in ('1', 'true', 'yes') if isinstance(v, str) else bool(v)

        self.use_fake = flag('use_fake_hardware', False)
        self.use_encoders = flag('use_encoders', True)
        self.odom_topic = str(p('odom_topic', 'odom').value)
        self.port = str(p('port', '/dev/ttyUSB0').value)
        self.baud = int(p('baud', 115200).value)
        self.wheel_radius = num('wheel_radius', 0.033)
        self.wheel_separation = num('wheel_separation', 0.160)
        self.ticks_per_rev = num('ticks_per_rev', 1320.0)
        self.max_wheel_speed = num('max_wheel_speed', 12.0)   # rad/s
        self.cmd_timeout = num('cmd_timeout', 0.5)            # s
        self.rate = num('rate', 30.0)                         # Hz
        self.publish_tf = flag('publish_tf', True)
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_footprint').value

        self.odom = DiffDriveOdometry(
            self.wheel_radius, self.wheel_separation, self.ticks_per_rev)

        self._lock = threading.Lock()
        self._ticks = None           # latest (left, right) from MCU
        self._fake_ticks = [0.0, 0.0]
        self._cmd = (0.0, 0.0)       # wheel rad/s
        self._last_cmd_time = 0.0
        self._serial = None
        self._reset_odom_ticks = False
        self._running = True

        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        if self.use_fake:
            self.get_logger().info('FAKE HARDWARE mode: no serial port, perfect odometry.')
        else:
            if not self.use_encoders:
                self.get_logger().info(
                    f'NO ENCODERS: open-loop drive, dead-reckoning on /{self.odom_topic}'
                    f' (TF {"on" if self.publish_tf else "off"})')
            if serial is None:
                raise RuntimeError('pyserial missing: sudo apt install python3-serial')
            self._reader = threading.Thread(target=self._serial_loop, daemon=True)
            self._reader.start()

        self._last_update = self.get_clock().now()
        self.create_timer(1.0 / self.rate, self._update)

    # ------------------------------------------------------------------ cmd_vel
    def _on_cmd_vel(self, msg):
        left, right = wheel_speeds_from_twist(
            msg.linear.x, msg.angular.z, self.wheel_radius, self.wheel_separation)
        left, right = scale_to_limit(left, right, self.max_wheel_speed)
        with self._lock:
            self._cmd = (left, right)
            self._last_cmd_time = time.monotonic()

    def _current_cmd(self):
        with self._lock:
            if time.monotonic() - self._last_cmd_time > self.cmd_timeout:
                return 0.0, 0.0   # safety stop when cmd_vel goes quiet
            return self._cmd

    # ------------------------------------------------------------------- serial
    def _serial_loop(self):
        """Open the port (retrying), then read encoder lines forever."""
        while self._running and rclpy.ok():
            if self._serial is None:
                try:
                    self._serial = serial.Serial(self.port, self.baud, timeout=0.1)
                    time.sleep(2.0)  # many boards reset when the port opens
                    self._serial.reset_input_buffer()
                    self.get_logger().info(f'Connected to MC 4.0 on {self.port}')
                except (serial.SerialException, OSError) as e:
                    self.get_logger().warn(
                        f'Cannot open {self.port} ({e}); retrying in 2 s',
                        throttle_duration_sec=10.0)
                    time.sleep(2.0)
                    continue
            try:
                line = self._serial.readline().decode('ascii', errors='ignore').strip()
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'Serial error: {e}; reconnecting')
                self._close_serial()
                continue
            self._parse_line(line)

    def _parse_line(self, line):
        parts = line.split()
        if len(parts) == 3 and parts[0] == 'E':
            try:
                ticks = (int(parts[1]), int(parts[2]))
            except ValueError:
                return
            with self._lock:
                self._ticks = ticks
        elif line.startswith('#'):
            self.get_logger().info(f'MCU: {line[1:].strip()}')

    def _send(self, left, right):
        port = self._serial  # local copy: the reader thread may reset it
        if port is None:
            return
        try:
            port.write(f'V {left:.3f} {right:.3f}\n'.encode('ascii'))
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial write failed: {e}')
            self._close_serial()

    def _close_serial(self):
        try:
            if self._serial is not None:
                self._serial.close()
        except Exception:
            pass
        self._serial = None
        with self._lock:
            self._ticks = None
            self._reset_odom_ticks = True  # avoid a jump when ticks restart after reconnect

    # ------------------------------------------------------------------- update
    def _update(self):
        now = self.get_clock().now()
        dt = (now - self._last_update).nanoseconds * 1e-9
        self._last_update = now
        left_cmd, right_cmd = self._current_cmd()

        if not self.use_fake:
            self._send(left_cmd, right_cmd)

        if self.use_fake or not self.use_encoders:
            # Integrate the commanded wheel speeds into virtual encoder ticks.
            k = self.ticks_per_rev / (2.0 * math.pi)
            self._fake_ticks[0] += left_cmd * dt * k
            self._fake_ticks[1] += right_cmd * dt * k
            ticks = (self._fake_ticks[0], self._fake_ticks[1])
        else:
            with self._lock:
                ticks = self._ticks
                if self._reset_odom_ticks:
                    self.odom._prev = None  # keep pose, re-baseline tick counts
                    self._reset_odom_ticks = False
            if ticks is None:
                return  # no data from MCU yet

        v, w = self.odom.update(ticks[0], ticks[1], dt)
        self._publish(now, v, w)

    def _publish(self, now, v, w):
        stamp = now.to_msg()
        qz = math.sin(self.odom.theta / 2.0)
        qw = math.cos(self.odom.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.odom.x
        odom.pose.pose.position.y = self.odom.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        # Diagonal covariances: x, y, z, roll, pitch, yaw
        pose_cov = [0.01, 0.01, 1e6, 1e6, 1e6, 0.03]
        twist_cov = [0.01, 1e6, 1e6, 1e6, 1e6, 0.03]
        for i in range(6):
            odom.pose.covariance[i * 7] = pose_cov[i]
            odom.twist.covariance[i * 7] = twist_cov[i]
        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.odom.x
            t.transform.translation.y = self.odom.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

        js = JointState()
        js.header.stamp = stamp
        # Rear joints exist only in the 4-wheel URDF; extra names are ignored otherwise.
        js.name = ['left_wheel_joint', 'right_wheel_joint',
                   'rear_left_wheel_joint', 'rear_right_wheel_joint']
        js.position = [self.odom.left_angle, self.odom.right_angle,
                       self.odom.left_angle, self.odom.right_angle]
        self.joint_pub.publish(js)

    def shutdown(self):
        self._running = False
        self._send(0.0, 0.0)
        self._close_serial()


def main(args=None):
    rclpy.init(args=args)
    node = Mc4BaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
