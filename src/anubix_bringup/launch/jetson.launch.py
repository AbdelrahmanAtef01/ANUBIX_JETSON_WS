#!/usr/bin/env python3
"""
ANUBIX Jetson Launch
=====================
Launches only the nodes that run on the Jetson Orin Nano:
  - anubix_master       (OmniLink AI agent bridge)
  - anubix_arm          (arm control placeholder)
  - anubix_spectrometer (spectral analysis)
  - anubix_jetson_bridge (cross-machine link monitor)

Navigation and Perception run on the Raspberry Pi.
Launch those with: ros2 launch anubix_bringup_rpi rpi_full.launch.py

Usage:
    ros2 launch anubix_bringup jetson.launch.py
    ros2 launch anubix_bringup jetson.launch.py simulate:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    simulate_arg = DeclareLaunchArgument(
        'simulate', default_value='true',
        description='Run hardware stacks in simulation/placeholder mode')

    master_share   = get_package_share_directory('anubix_master')
    arm_share      = get_package_share_directory('anubix_arm')
    spectro_share  = get_package_share_directory('anubix_spectrometer')
    bridge_share   = get_package_share_directory('anubix_jetson_bridge')
    supabase_share = get_package_share_directory('anubix_supabase')

    master_node = Node(
        package='anubix_master',
        executable='master_node',
        name='anubix_master',
        parameters=[os.path.join(master_share, 'config', 'master_params.yaml')],
        output='screen',
        emulate_tty=True,
    )

    arm_node = Node(
        package='anubix_arm',
        executable='arm_node',
        name='anubix_arm',
        parameters=[os.path.join(arm_share, 'config', 'arm_params.yaml')],
        output='screen',
        emulate_tty=True,
    )

    spectro_node = Node(
        package='anubix_spectrometer',
        executable='spectrometer_node',
        name='anubix_spectrometer',
        parameters=[os.path.join(spectro_share, 'config', 'spectrometer_params.yaml')],
        output='screen',
        emulate_tty=True,
    )

    bridge_node = Node(
        package='anubix_jetson_bridge',
        executable='jetson_bridge_node',
        name='anubix_jetson_bridge',
        parameters=[os.path.join(bridge_share, 'config', 'jetson_bridge_params.yaml')],
        output='screen',
        emulate_tty=True,
    )

    supabase_node = Node(
        package='anubix_supabase',
        executable='supabase_node',
        name='anubix_supabase_uploader',
        parameters=[os.path.join(supabase_share, 'config', 'supabase_params.yaml')],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        simulate_arg,
        master_node,
        arm_node,
        spectro_node,
        bridge_node,
        supabase_node,
    ])
