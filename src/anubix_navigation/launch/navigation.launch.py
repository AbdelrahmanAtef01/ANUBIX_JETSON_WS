#!/usr/bin/env python3
"""Launch the ANUBIX navigation node (Raspberry Pi)."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('anubix_navigation'),
        'config',
        'nav_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='anubix_navigation',
            executable='nav_node',
            name='anubix_navigation',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        ),
    ])
