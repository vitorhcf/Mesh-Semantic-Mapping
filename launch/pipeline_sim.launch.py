#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_semantic_publisher = LaunchConfiguration('use_semantic_publisher')

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
        condition=IfCondition(use_semantic_publisher),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_semantic_publisher',
            default_value='false',
            description='Launch the legacy STL republisher node.',
        ),
        vision_snapshot,
        processing_node,
        semantic_publisher,
    ])
