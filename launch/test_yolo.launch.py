#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    yolo_tester = Node(
        package='mesh_semantic_mapping',
        executable='test_yolo.py',
        name='yolo_image_tester',
        output='screen',
    )

    return LaunchDescription([
        yolo_tester,
    ])
