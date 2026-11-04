"""Autonomous navigation (Nav2) for the MC 4.0.

Navigate while mapping (no saved map needed):
    ros2 launch mc4_bringup navigation.launch.py slam:=true
Navigate in a saved map (AMCL localisation):
    ros2 launch mc4_bringup navigation.launch.py map:=$HOME/maps/arena.yaml

Then click "2D Goal Pose" in RViz, or run mc4_autonomy waypoint_patrol.

The Nav2 servers are started directly (not via nav2_bringup) so only what a
small robot needs is launched and the setup works the same on Humble/Jazzy.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

NAV_NODES = [
    'controller_server',
    'planner_server',
    'behavior_server',
    'velocity_smoother',
    'bt_navigator',
    'waypoint_follower',
]


def generate_launch_description():
    bringup = FindPackageShare('mc4_bringup')
    params = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    common = [params, {'use_sim_time': use_sim_time}]

    args = [
        DeclareLaunchArgument('slam', default_value='false',
                              description='true: map with slam_toolbox while navigating'),
        DeclareLaunchArgument('map', default_value='',
                              description='Full path to a saved map .yaml (when slam:=false)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file',
                              default_value=PathJoinSubstitution([bringup, 'config', 'nav2.yaml'])),
    ]

    # --- Localisation: either SLAM, or map_server + AMCL -------------------
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, 'launch', 'slam.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(slam))

    localization = GroupAction(condition=UnlessCondition(slam), actions=[
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=common + [{'yaml_filename': LaunchConfiguration('map')}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             output='screen', parameters=common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': ['map_server', 'amcl']}]),
    ])

    # --- Planning & control ------------------------------------------------
    # controller/behaviors publish cmd_vel_nav; velocity_smoother turns it into /cmd_vel.
    nav = [
        Node(package='nav2_controller', executable='controller_server', output='screen',
             parameters=common, remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=common),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=common, remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=common,
             remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=common),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen', parameters=common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': NAV_NODES}]),
    ]

    return LaunchDescription(args + [slam_launch, localization] + nav)
