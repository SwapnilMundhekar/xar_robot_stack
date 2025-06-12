#!/usr/bin/env python3
"""
nav_manager.launch.py
• slam:=true           → run SLAM (SLAM Toolbox) and ignore ‘map’
• slam:=false map:=…   → load an existing map YAML
• Starts:
    – rosbridge_websocket  (ws://192.168.100.11:9090)
    – TF republisher       (/tf streamed, not latched)
    – EKF  (/odometry/filtered)  ← wheel odom + IMU
    – Nav2 bring-up         (costmaps, BT navigator…)
    – Your XAR behaviour nodes  (nav_manager, spin_driver, battery_pub)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ─────────── launch-time parameters ─────────────────────────────
    slam_arg = DeclareLaunchArgument(
        'slam', default_value='true',
        description='true = run SLAM, false = load a static map')

    map_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='Absolute path to a map YAML when slam:=false')

    # ─────────── rosbridge (WebSocket for HoloLens) ─────────────────
    rosbridge_ws = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090}]           # ← ws://192.168.100.11:9090
    )

    # ─────────── TF republisher  (latched → streaming) ──────────────
    tf_repub = Node(
        package='tf2_ros',
        executable='republish',
        name='tf_republisher',
        output='log',
        arguments=[
            'tf2_msgs/msg/TFMessage',
            'geometry_msgs/msg/TransformStamped',
            '/tf', '/tf'
        ]
    )

    # ─────────── Generic EKF  (/odometry/filtered) ──────────────────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            # Core
            'frequency':                        30.0,
            'sensor_timeout':                   0.1,
            'two_d_mode':                       True,
            'publish_tf':                       True,

            # Frames
            'map_frame':                        'map',
            'odom_frame':                       'odom',
            'base_link_frame':                  'base_link',
            'world_frame':                      'odom',

            # Inputs ─ adjust if your topics differ
            'odom0':                            '/rosbot_base_controller/odom',
            'odom0_differential':               False,
            'odom0_relative':                   False,

            'imu0':                             '/bno055/imu',
            'imu0_differential':                False,
            'imu0_relative':                    False,
            'imu0_remove_gravitational_acceleration': True
        }]
    )

    # ─────────── Nav2 bring-up (SLAM or static map) ────────────────
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart':    'true',
            'slam':         LaunchConfiguration('slam'),
            'map':          LaunchConfiguration('map'),
        }.items())

    # ─────────── XAR behaviour nodes ───────────────────────────────
    nav_manager_node = Node(
        package='xar_nav_manager', executable='nav_manager',
        name='nav_manager', output='screen')

    spin_driver_node = Node(
        package='xar_nav_manager', executable='spin_driver',
        name='spin_driver', output='screen')

    battery_pub_node = Node(
        package='xar_nav_manager', executable='battery_publisher',
        name='battery_publisher', output='screen')

    # ─────────── assemble launch description ───────────────────────
    return LaunchDescription([
        slam_arg,
        map_arg,
        rosbridge_ws,
        tf_repub,
        ekf_node,
        nav2_bringup,
        nav_manager_node,
        spin_driver_node,
        battery_pub_node,
    ])

