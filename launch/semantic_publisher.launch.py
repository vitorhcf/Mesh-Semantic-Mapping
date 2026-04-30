#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    semantic_publisher = Node(
        package='mesh_semantic_mapping',
        executable='semantic_publisher.py',
        name='semantic_map_publisher',
        output='screen',
    )

    return LaunchDescription([
        semantic_publisher,
    ])
