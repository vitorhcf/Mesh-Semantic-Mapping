#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    
    # 1. FORÇAR A REDE ROBUSTA EM TODOS OS PROCESSOS DESTE LAUNCH FILE
    set_dds = SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_cyclonedds_cpp')

    # 2. Injetar a gravação (agora com 'warn' em minúsculas)
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', 'rosbag2_2026_05_25-17_06_03/', '--loop', '--log-level', 'warn'],
        cwd='/ros2_ws/bag',
        output='screen',
    )

    # 3. Arrancar a Visualização
    rviz = ExecuteProcess(
        cmd=['rviz2', '-d', '/ros2_ws/mesh.rviz', '--ros-args', '--log-level', 'WARN'],
        output='screen',
    )

    # 4. Arrancar a Ponte SAM3 (sem flags inválidas)
    sam3_interface = ExecuteProcess(
        cmd=['ros2', 'launch', 'robot_perception_interfaces_bringup', 'sam3_interface.launch.py'],
        output='screen',
    )

    # 5. Arrancar a Pipeline (sem flags inválidas)
    pipeline = ExecuteProcess(
        cmd=['ros2', 'launch', 'mesh_semantic_mapping', 'pipeline.launch.py'],
        output='screen',
    )

    return LaunchDescription([
        set_dds,
        bag_play,
        RegisterEventHandler(
            OnProcessStart(
                target_action=bag_play,
                on_start=[rviz],
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=rviz,
                on_start=[sam3_interface],
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=sam3_interface,
                on_start=[pipeline],
            )
        ),
    ])