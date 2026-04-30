#!/usr/bin/env python3
"""
Example usage and testing script for mesh_semantic_mapping package.
This demonstrates how to run each component of the package.
"""

import subprocess
import sys
import os


def print_header(msg):
    print("\n" + "="*60)
    print(f"  {msg}")
    print("="*60 + "\n")


def run_command(cmd, description):
    """Run a shell command and display output."""
    print(f"Running: {description}")
    print(f"Command: {cmd}\n")
    result = subprocess.run(cmd, shell=True)
    return result.returncode == 0


def main():
    print_header("Mesh Semantic Mapping - Package Setup and Usage")
    
    ws_root = "/mnt/ssd/pcl/ros2_ws"
    
    # Step 1: Build
    print_header("Step 1: Building the Package")
    print("Building mesh_semantic_mapping package...")
    success = run_command(
        f"cd {ws_root} && colcon build --packages-select mesh_semantic_mapping",
        "Build package"
    )
    
    if not success:
        print("❌ Build failed!")
        sys.exit(1)
    
    print("✅ Build succeeded!")
    
    # Step 2: Source setup
    print_header("Step 2: Sourcing ROS2 Environment")
    print("Run the following command in your terminal:")
    print(f"  source {ws_root}/install/setup.bash\n")
    
    # Step 3: Available commands
    print_header("Step 3: Available Commands")
    
    commands = {
        "Semantic Publisher": {
            "launch": "ros2 launch mesh_semantic_mapping semantic_publisher.launch.py",
            "direct": "ros2 run mesh_semantic_mapping semantic_publisher.py",
            "description": "Publishes mesh markers to /semantic_map_meshes for RViz visualization"
        },
        "YOLO Image Tester": {
            "launch": "ros2 launch mesh_semantic_mapping test_yolo.launch.py",
            "direct": "ros2 run mesh_semantic_mapping test_yolo.py",
            "description": "Detects tables and chairs in camera images (saves as visao_do_robo_segmentada.jpg)"
        },
        "Vision Snapshot": {
            "launch": "ros2 launch mesh_semantic_mapping vision_snapshot.launch.py",
            "direct": "ros2 run mesh_semantic_mapping vision_snapshot.py",
            "description": "Captures RGB-D data and performs semantic segmentation"
        },
        "Processing Script": {
            "direct": "python3 scripts/processing.py",
            "description": "Converts PLY point clouds to STL mesh files (requires PLY input file)"
        },
    }
    
    for i, (name, info) in enumerate(commands.items(), 1):
        print(f"\n{i}. {name}")
        print(f"   Description: {info['description']}")
        if 'launch' in info:
            print(f"   Launch:     {info['launch']}")
        print(f"   Direct:     {info['direct']}")
    
    # Step 4: Prerequisites
    print_header("Step 4: Prerequisites Checklist")
    
    prerequisites = {
        "Python packages": "pip install open3d ultralytics numpy opencv-python",
        "ROS2 dependencies": "sudo apt-get install ros-humble-sensor-msgs ros-humble-visualization-msgs ros-humble-cv-bridge ros-humble-image-transport ros-humble-message-filters ros-humble-tf2-ros",
        "System dependencies": "sudo apt-get install libopencv-dev python3-opencv",
    }
    
    for name, cmd in prerequisites.items():
        print(f"□ {name}")
        print(f"   Command: {cmd}\n")
    
    # Step 5: File paths
    print_header("Step 5: Important File Paths")
    
    paths = {
        "Package root": f"{ws_root}/src/mesh_semantic_mapping",
        "Scripts": f"{ws_root}/src/mesh_semantic_mapping/scripts",
        "Launch files": f"{ws_root}/src/mesh_semantic_mapping/launch",
        "Expected mesh files": "/ros2_ws/table_mesh.stl, /ros2_ws/chair_mesh.stl",
        "YOLO model": f"{ws_root}/src/mesh_semantic_mapping/scripts/yolov8n-seg.pt",
    }
    
    for name, path in paths.items():
        print(f"{name:.<30} {path}")
    
    # Step 6: Useful commands
    print_header("Step 6: Useful ROS2 Commands")
    
    useful_commands = [
        ("Monitor topics", "ros2 topic list"),
        ("Echo topic messages", "ros2 topic echo /semantic_map_meshes"),
        ("Check node info", "ros2 node info /semantic_map_publisher"),
        ("List packages", "ros2 pkg list | grep mesh_semantic_mapping"),
        ("View logs", "ros2 run mesh_semantic_mapping semantic_publisher.py 2>&1 | head -20"),
    ]
    
    for description, cmd in useful_commands:
        print(f"{description:.<35} {cmd}")
    
    # Step 7: Quick start
    print_header("Quick Start Example")
    
    quick_start = """
# Terminal 1: Build and source
cd /mnt/ssd/pcl/ros2_ws
colcon build --packages-select mesh_semantic_mapping
source install/setup.bash

# Terminal 2: Start the semantic publisher (with RViz running)
ros2 launch mesh_semantic_mapping semantic_publisher.launch.py

# Terminal 3: In RViz, add a MarkerArray visualization for topic: /semantic_map_meshes

# Terminal 4: Or test YOLO detection (requires camera stream)
ros2 launch mesh_semantic_mapping test_yolo.launch.py
"""
    
    print(quick_start)
    
    print_header("Setup Complete!")
    print("✅ Your mesh_semantic_mapping package is ready to use!")
    print("\nNext steps:")
    print("1. Source the ROS2 environment: source install/setup.bash")
    print("2. Choose a node to run from Step 3 above")
    print("3. For visualization, ensure RViz2 is running: rviz2")


if __name__ == '__main__':
    main()
