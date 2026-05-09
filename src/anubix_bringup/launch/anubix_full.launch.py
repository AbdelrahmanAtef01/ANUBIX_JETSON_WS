#!/usr/bin/env python3
"""
ANUBIX Full System Launch
==========================
Launches the master node + all 4 stacks on the Jetson Orin Nano.

Usage:
    ros2 launch anubix_bringup anubix_full.launch.py

With simulation mode (no hardware needed):
    ros2 launch anubix_bringup anubix_full.launch.py simulate:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    simulate_arg = DeclareLaunchArgument(
        'simulate', default_value='true',
        description='Run stacks in simulation/placeholder mode')

    # Package share directories
    master_share = get_package_share_directory('anubix_master')
    nav_share = get_package_share_directory('anubix_navigation')
    perception_share = get_package_share_directory('anubix_perception')
    arm_share = get_package_share_directory('anubix_arm')
    spectro_share = get_package_share_directory('anubix_spectrometer')

    # Master node
    master_node = Node(
        package='anubix_master',
        executable='master_node',
        name='anubix_master',
        parameters=[os.path.join(master_share, 'config', 'master_params.yaml')],
        output='screen',
    )

    # Navigation stack
    nav_node = Node(
        package='anubix_navigation',
        executable='nav_node',
        name='anubix_navigation',
        parameters=[os.path.join(nav_share, 'config', 'nav_params.yaml')],
        output='screen',
    )

    # Perception stack
    perception_node = Node(
        package='anubix_perception',
        executable='perception_node',
        name='anubix_perception',
        parameters=[os.path.join(perception_share, 'config', 'perception_params.yaml')],
        output='screen',
    )

    # Arm control stack
    arm_node = Node(
        package='anubix_arm',
        executable='arm_node',
        name='anubix_arm',
        parameters=[os.path.join(arm_share, 'config', 'arm_params.yaml')],
        output='screen',
    )

    # Spectrometer stack
    spectro_node = Node(
        package='anubix_spectrometer',
        executable='spectrometer_node',
        name='anubix_spectrometer',
        parameters=[os.path.join(spectro_share, 'config', 'spectrometer_params.yaml')],
        output='screen',
    )

    return LaunchDescription([
        simulate_arg,
        master_node,
        nav_node,
        perception_node,
        arm_node,
        spectro_node,
    ])
