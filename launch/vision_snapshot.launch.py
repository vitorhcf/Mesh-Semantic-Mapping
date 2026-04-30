#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    vision_snapshot = Node(
        package='mesh_semantic_mapping',
        executable='vision_snapshot.py',
        name='tiago_vision_snapshot',
        output='screen',
    )

    return LaunchDescription([
        vision_snapshot,
    ])
