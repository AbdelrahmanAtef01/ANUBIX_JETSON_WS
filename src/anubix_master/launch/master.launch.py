#!/usr/bin/env python3
"""Launch the ANUBIX master node."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('anubix_master'),
        'config',
        'master_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='anubix_master',
            executable='master_node',
            name='anubix_master',
            parameters=[config],
            output='screen',
        ),
    ])
