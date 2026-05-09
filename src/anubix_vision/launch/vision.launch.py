#!/usr/bin/env python3
"""Launch the ANUBIX Vision node standalone (useful for isolated testing)."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('anubix_vision'),
        'config',
        'vision_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='anubix_vision',
            executable='vision_node',
            name='anubix_vision',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        ),
    ])
