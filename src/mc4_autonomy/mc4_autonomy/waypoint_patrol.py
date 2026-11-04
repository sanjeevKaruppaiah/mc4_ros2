"""Autonomous waypoint patrol for the MC 4.0 using Nav2.

Nav2 plans a collision-free path to every waypoint (global planner),
follows it (local controller) and re-plans around new obstacles seen by the
lidar. This script only decides WHERE to go.

    ros2 run mc4_autonomy waypoint_patrol --ros-args \
        --params-file $(ros2 pkg prefix mc4_autonomy)/share/mc4_autonomy/config/waypoints.yaml
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(nav, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    rclpy.init()
    nav = BasicNavigator(node_name='waypoint_patrol')

    flat = nav.declare_parameter('waypoints', [1.0, 0.0, 0.0]).value
    loops = nav.declare_parameter('loops', 1).value
    use_slam = nav.declare_parameter('use_slam', True).value
    init = nav.declare_parameter('initial_pose', [0.0, 0.0, 0.0]).value

    if len(flat) % 3 != 0:
        nav.get_logger().error('waypoints must be a flat list of x, y, yaw triples')
        return
    route = [flat[i:i + 3] for i in range(0, len(flat), 3)]

    if use_slam:
        # slam_toolbox provides map->odom; there is no AMCL node to wait for.
        # 'robot_localization' tells BasicNavigator to skip the localizer check.
        nav.waitUntilNav2Active(localizer='robot_localization')
    else:
        nav.setInitialPose(make_pose(nav, *init))
        nav.waitUntilNav2Active(localizer='amcl')

    lap = 0
    while rclpy.ok() and (loops == 0 or lap < loops):
        lap += 1
        poses = [make_pose(nav, *wp) for wp in route]
        nav.get_logger().info(f'Lap {lap}: following {len(poses)} waypoints')
        nav.followWaypoints(poses)

        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb is not None:
                nav.get_logger().info(
                    f'  heading to waypoint {fb.current_waypoint + 1}/{len(poses)}',
                    throttle_duration_sec=3.0)

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            nav.get_logger().info(f'Lap {lap} complete')
        elif result == TaskResult.CANCELED:
            nav.get_logger().warn('Patrol cancelled')
            break
        else:
            nav.get_logger().error(f'Lap {lap} failed; retrying next lap')

    nav.get_logger().info('Patrol finished')
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
