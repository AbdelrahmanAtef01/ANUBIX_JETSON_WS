#!/usr/bin/env python3
"""Launch the ANUBIX Supabase uploader node (standalone)."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('anubix_supabase'),
        'config',
        'supabase_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='anubix_supabase',
            executable='supabase_node',
            name='anubix_supabase_uploader',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        ),
    ])
