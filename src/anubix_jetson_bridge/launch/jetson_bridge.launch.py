#!/usr/bin/env python3
"""Launch the ANUBIX Jetson Bridge monitoring node."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('anubix_jetson_bridge'),
        'config',
        'jetson_bridge_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='anubix_jetson_bridge',
            executable='jetson_bridge_node',
            name='anubix_jetson_bridge',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        ),
    ])
