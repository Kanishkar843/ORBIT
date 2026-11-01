#!/usr/bin/env python3
"""
Real-Hardware Navigation Launch (A* only)
==========================================
Production launch file for the physical diffbot robot. Simplified from the
original multi-planner (A* / RRT / dual / D*-ACO) research launch down to a
single, production A* planner — the RRT and D*-ACO code paths, and the
planner-comparison research nodes (performance_analyzer, result_visualizer,
dstar_aco_visualizer, rrt_planner) have been removed entirely.

Prerequisites (must already be running before this launch):
  Terminal 1:  ros2 launch zlac8015d_driver hardware_bringup.launch.py
               (starts robot_state_publisher + motor odometry + odom->base_footprint TF)
  Terminal 2:  ros2 launch rplidar_ros rplidar_c1_launch.py frame_id:=lidar
               (publishes raw /scan)

Note: robot_state_publisher is intentionally NOT started here — it is owned
by hardware_bringup.launch.py. Running a second instance publishes
duplicate/conflicting TF and causes "extrapolation into the future" errors
in Nav2.

Startup order:
  t=0s   laser_filters (masks blocked rear arc, /scan -> /scan_filtered)
  t=5s   map_server, amcl, planner_server (lifecycle_manager_localization)
  t=6s   controller_server, rviz2
  t=7s   recoveries_server (behavior_server)
  t=8s   bt_navigator
  t=10s  lifecycle_manager_navigation

Usage:
  ros2 launch path_planning_pkg navigation_real.launch.py
  ros2 launch path_planning_pkg navigation_real.launch.py map:=/path/to/mymap.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name = 'path_planning_pkg'
    pkg_dir = get_package_share_directory(pkg_name)

    config_dir = os.path.join(pkg_dir, 'config')
    maps_dir = os.path.join(pkg_dir, 'maps')
    rviz_dir = os.path.join(pkg_dir, 'rviz')

    use_sim_time = False  # always False for real hardware

    # -- Launch arguments -------------------------------------------------
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(maps_dir, 'map_1777558867.yaml'),
        description='Full path to map yaml file to load',
    )

    map_yaml_file = LaunchConfiguration('map')

    # -- Laser filters: mask the chassis-blocked rear arc ------------------
    # Subscribes /scan, publishes /scan_filtered. Every downstream consumer
    # (AMCL, global/local costmaps) is configured to use /scan_filtered.
    laser_filters_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filters',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'laser_filters.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    # -- Map server ----------------------------------------------------------
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'yaml_filename': map_yaml_file,
                'use_sim_time': use_sim_time,
                'frame_id': 'map',
                'topic_name': 'map',
            },
        ],
    )

    # -- AMCL localisation -----------------------------------------------------
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'amcl.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('scan', '/scan_filtered')],
    )

    lifecycle_manager_localization = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']},
                    {'bond_timeout': 4.0},
                ],
            )
        ],
    )

    # -- Planner server (A* only) ----------------------------------------------
    planner_server = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[
                    os.path.join(config_dir, 'nav2_astar_planning.yaml'),
                    {'use_sim_time': use_sim_time},
                ],
            ),
        ],
    )

    # -- Controller server -----------------------------------------------------
    controller_server = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[
                    os.path.join(config_dir, 'nav2_controller.yaml'),
                    {'use_sim_time': use_sim_time},
                ],
                remappings=[('cmd_vel', 'cmd_vel')],
            )
        ],
    )

    # -- Recoveries / behavior server -------------------------------------------
    recoveries_server = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='recoveries_server',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'local_costmap_topic': 'local_costmap/costmap'},
                    {'local_footprint_topic': 'local_costmap/published_footprint'},
                    {'cycle_frequency': 10.0},
                    {'behavior_plugins': ['spin', 'backup', 'wait']},
                    {'spin': {'plugin': 'nav2_behaviors/Spin'}},
                    {'backup': {'plugin': 'nav2_behaviors/BackUp'}},
                    {'wait': {'plugin': 'nav2_behaviors/Wait'}},
                ],
            )
        ],
    )

    # -- BT Navigator ------------------------------------------------------------
    bt_navigator = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[
                    os.path.join(config_dir, 'nav2_bt.yaml'),
                    {'use_sim_time': use_sim_time},
                ],
            )
        ],
    )

    # -- Lifecycle manager -- navigation -----------------------------------------
    lifecycle_manager_navigation = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': [
                        'planner_server',
                        'controller_server',
                        'recoveries_server',
                        'bt_navigator',
                    ]},
                    {'bond_timeout': 4.0},
                ],
            )
        ],
    )

    # -- RViz ----------------------------------------------------------------
    rviz_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', os.path.join(rviz_dir, 'rviz_navigation.rviz')],
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ],
    )

    return LaunchDescription([
        declare_map_arg,
        # NOTE: no robot_state_publisher here — owned by hardware_bringup.launch.py
        laser_filters_node,
        map_server,
        amcl,
        lifecycle_manager_localization,
        planner_server,
        controller_server,
        recoveries_server,
        bt_navigator,
        lifecycle_manager_navigation,
        rviz_node,
    ])
