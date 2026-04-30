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

    processing_node = Node(
        package='mesh_semantic_mapping',
        executable='processing_node.py',
        name='processing_node',
        output='screen',
    )

    semantic_publisher = Node(
        package='mesh_semantic_mapping',
        executable='semantic_publisher.py',
        name='semantic_map_publisher',
        output='screen',
    )

    return LaunchDescription([
        vision_snapshot,
        processing_node,
        semantic_publisher,
    ])