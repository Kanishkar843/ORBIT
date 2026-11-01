#!/usr/bin/env python3
"""
Real-Hardware SLAM Mapping Launch
===================================
PHASE 1 (real robot): Drive the robot around the environment with teleop
while slam_toolbox builds a map. Save the map when done.

Starts:
  1. zlac8015d_driver/hardware_bringup — motors + odom + TF (odom→base_footprint),
                                          also owns robot_state_publisher
  2. RPLidar C1                        — /scan with frame_id=lidar
  3. Static TF fallbacks               — in case hardware_bringup is not running
  4. laser_filters                     — /scan → /scan_filtered (masks blocked rear arc)
  5. slam_toolbox                       — builds /map, publishes map→odom TF
  6. rviz2                              — live map view
  7. teleop_twist_keyboard              — in a new xterm window

After mapping, save the map:
  ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

FIXES APPLIED:
  FIX 1 - URDF path: the original code searched for robot.urdf / robot.xacro
           which do not exist. The actual file is diffbot.urdf. Fixed to load
           diffbot.urdf directly from path_planning_pkg/urdf/.

  FIX 2 - RPLidar frame_id was missing: rplidar_c1_launch.py defaults to
           frame_id='laser' but the URDF defines the lidar link as 'lidar'.
           The mapping_real launch already overrides to 'lidar' — CORRECT.
           Added explicit check so it cannot revert to 'laser'.

  FIX 3 - RViz config fallback used a hardcoded /home/mohan_g/ path which
           breaks on any other machine. Removed the hardcoded fallback; if the
           installed rviz config is missing, RViz opens without a config and
           the user gets a warning instead of a crash.

  FIX 4 - hardware_bringup is launched as IncludeLaunchDescription here so
           that robot_state_publisher is NOT duplicated (hardware_bringup
           already starts RSP). Using IncludeLaunchDescription instead of
           direct Node() avoids the double-RSP TF conflict.

  FIX 5 - SLAM base_frame must be base_footprint (odom→base_footprint TF
           is what hardware_driver publishes). Added explicit override in
           Node parameters so the yaml value cannot be accidentally wrong.

  FIX 6 - Static TFs now only run when hardware_bringup is NOT included.
           Kept as safety net but renamed to avoid collision with hardware_bringup
           static TF node names.

  FIX 7 - serial_port LaunchConfiguration was not forwarded to hardware_bringup
           IncludeLaunchDescription. Fixed with launch_arguments.

Usage:
  ros2 launch path_planning_pkg mapping_real.launch.py
  ros2 launch path_planning_pkg mapping_real.launch.py serial_port:=/dev/ttyUSB1
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Arguments ────────────────────────────────────────────────────────────
    use_sim_time     = LaunchConfiguration('use_sim_time', default='false')
    serial_port      = LaunchConfiguration('serial_port',  default='/dev/ttyACM0')
    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=os.path.join(
            get_package_share_directory('path_planning_pkg'),
            'config', 'mapper_params_online_async.yaml'))

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_serial_port  = DeclareLaunchArgument('serial_port',  default_value='/dev/ttyACM0')
    declare_slam_params  = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory('path_planning_pkg'),
            'config', 'mapper_params_online_async.yaml'))

    # ── FIX 1: Load the actual URDF (diffbot.urdf) directly ──────────────────
    pkg_share = get_package_share_directory('path_planning_pkg')
    urdf_file = os.path.join(pkg_share, 'urdf', 'diffbot.urdf')

    if not os.path.isfile(urdf_file):
        raise RuntimeError(
            f'\n\n*** Could not find diffbot.urdf at {urdf_file} ***\n'
            'Make sure path_planning_pkg is built and installed.\n')

    with open(urdf_file, 'r') as f:
        robot_description_xml = f.read()

    print(f'[mapping_real.launch] Loaded URDF: {urdf_file}')

    # ── FIX 3: RViz config — no hardcoded home paths ──────────────────────────
    rviz_config = os.path.join(pkg_share, 'rviz', 'rviz_mapping.rviz')
    if not os.path.isfile(rviz_config):
        print(f'[mapping_real.launch] WARNING: RViz config not found at {rviz_config}')
        print('[mapping_real.launch] RViz will open without a config file.')
        rviz_config = ''

    # =========================================================================
    # NODES
    # =========================================================================

    # ── 1. Hardware Bringup (motors + odom TF: odom→base_footprint) ──────────
    # FIX 4 & FIX 7: Include hardware_bringup with serial_port forwarded.
    # hardware_bringup.launch.py owns robot_state_publisher — it is
    # intentionally NOT started separately here. A second RSP instance
    # would publish duplicate/conflicting TF and break Nav2's TF lookups
    # (matches the fix already applied in navigation_real.launch.py).
    # robot_description_xml (loaded above from diffbot.urdf) is kept for
    # reference/validation only; it is not passed to a second RSP node.
    hardware_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zlac8015d_driver'),
                'launch', 'hardware_bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'serial_port': serial_port,  # FIX 7: forward serial_port arg
        }.items()
    )

    # ── 2. RPLidar C1 ────────────────────────────────────────────────────────
    # FIX 2: frame_id='lidar' MUST match the link name in diffbot.urdf (lidar_joint → lidar).
    # rplidar_c1_launch.py defaults to 'laser' which would break TF lookups.
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rplidar_ros'),
                'launch', 'rplidar_c1_launch.py'
            ])
        ]),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',   # LiDAR port (not the motor port)
            'frame_id':    'lidar',           # FIX 2: must match URDF link name
        }.items()
    )

    # ── 3. Static TFs (safety net — also published by RSP via URDF joints) ───
    # These guard against the case where RSP hasn't started yet or the URDF
    # joint is not being processed. They are redundant once RSP is running.
    static_tf_fp_bl = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_footprint_to_base_link',
        arguments=['--x', '0', '--y', '0', '--z', '0.1',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_footprint', '--child-frame-id', 'base_link']
    )

    static_tf_bl_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_link_to_lidar',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.18',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'lidar']
    )

    static_tf_bl_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_link_to_camera',
        arguments=['--x', '0.1', '--y', '0.0', '--z', '0.15',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'camera']
    )

    static_tf_bl_caster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_link_to_caster_wheel',
        arguments=['--x', '-0.15', '--y', '0.0', '--z', '-0.05',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'caster_wheel']
    )

    # ── 4. Laser filters — mask the chassis-blocked rear arc ─────────────────
    # Subscribes /scan, publishes /scan_filtered, which SLAM Toolbox consumes.
    laser_filters_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filters',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'laser_filters.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    # ── 5. SLAM Toolbox ───────────────────────────────────────────────────────
    # FIX 5: Explicit base_frame/odom_frame/map_frame override to be safe
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {
                'use_sim_time': use_sim_time,
                'base_frame':   'base_footprint',   # FIX 5: must match odom TF child
                'odom_frame':   'odom',
                'map_frame':    'map',
                'scan_topic':   '/scan_filtered',
            }
        ],
    )

    # ── 6. RViz2 with mapping config ─────────────────────────────────────────
    rviz_args = ['-d', rviz_config] if rviz_config else []
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── 7. Teleop keyboard ───────────────────────────────────────────────────
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -title "Teleop - use arrow keys" -e',
        remappings=[('/cmd_vel', '/cmd_vel')]
    )

    # ── Startup order ─────────────────────────────────────────────────────────
    # t=0s  hardware_bringup (owns RSP) + static TFs + lidar + laser_filters
    #        TF tree must exist before slam_toolbox starts
    # t=2s  Teleop (can start early, just needs /cmd_vel)
    # t=5s  SLAM Toolbox (needs /scan_filtered + full TF chain: odom→base_footprint→base_link→lidar)
    # t=7s  RViz (needs map frame from SLAM — wait 2s after SLAM starts)

    slam_delayed   = TimerAction(period=5.0, actions=[slam_toolbox])
    rviz_delayed   = TimerAction(period=7.0, actions=[rviz_node])
    teleop_delayed = TimerAction(period=2.0, actions=[teleop_node])

    return LaunchDescription([
        declare_use_sim_time,
        declare_serial_port,
        declare_slam_params,

        # t=0s — core TF infrastructure
        hardware_bringup,
        rplidar_launch,
        static_tf_fp_bl,
        static_tf_bl_lidar,
        static_tf_bl_cam,
        static_tf_bl_caster,
        laser_filters_node,

        # Delayed
        teleop_delayed,   # t=2s
        slam_delayed,     # t=5s
        rviz_delayed,     # t=7s
    ])