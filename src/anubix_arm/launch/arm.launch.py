#!/usr/bin/env python3
"""Launch the ANUBIX arm control stack."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('anubix_arm'),
        'config',
        'arm_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='anubix_arm',
            executable='arm_node',
            name='anubix_arm',
            parameters=[config],
            output='screen',
        ),
    ])
