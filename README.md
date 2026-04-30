<<<<<<< HEAD
# Mesh Semantic Mapping

A ROS2 package for semantic mesh mapping and robot perception using YOLO object detection.

## Features

- **Semantic Publisher**: Publishes semantic mesh markers for visualization in RViz
- **YOLO Object Detection**: Detects tables and chairs in camera images
- **Vision Snapshot**: Captures and processes depth+RGB data with semantic segmentation
- **Mesh Processing**: Converts point clouds to STL mesh files

## Prerequisites

### System Dependencies

```bash
sudo apt-get update
sudo apt-get install -y \
  python3-pip \
  libopencv-dev \
  python3-opencv
```

### Python Dependencies

Install required Python packages:

```bash
pip install open3d ultralytics numpy opencv-python
```

### ROS2 Dependencies

These are automatically resolved through package.xml, but you can install them manually:

```bash
sudo apt-get install -y \
  ros-humble-sensor-msgs \
  ros-humble-visualization-msgs \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-message-filters \
  ros-humble-tf2-ros
```

## Building

Build the package from the workspace root:

```bash
cd /mnt/ssd/pcl/ros2_ws
colcon build --packages-select mesh_semantic_mapping
source install/setup.bash
```

## Running the Nodes

### 1. Semantic Publisher

Publishes mesh markers to `/semantic_map_meshes` topic:

```bash
# Using launch file
ros2 launch mesh_semantic_mapping semantic_publisher.launch.py

# Or directly
ros2 run mesh_semantic_mapping semantic_publisher.py
```

Expected files:
- `/ros2_ws/table_mesh.stl`
- `/ros2_ws/chair_mesh.stl`

### 2. YOLO Image Tester

Captures an image from the camera and runs YOLO detection:

```bash
# Using launch file
ros2 launch mesh_semantic_mapping test_yolo.launch.py

# Or directly
ros2 run mesh_semantic_mapping test_yolo.py
```

Subscribes to: `/head_front_camera/rgb/image_raw`
Output: `visao_do_robo_segmentada.jpg`

### 3. Vision Snapshot

Captures synchronized RGB-D data and performs semantic segmentation:

```bash
# Using launch file
ros2 launch mesh_semantic_mapping vision_snapshot.launch.py

# Or directly
ros2 run mesh_semantic_mapping vision_snapshot.py
```

Subscribes to:
- `/head_front_camera/rgb/image_raw`
- `/head_front_camera/depth/image_raw`
- `/head_front_camera/rgb/camera_info`

### 4. Processing Scripts

Process point clouds to generate meshes:

```bash
# Standalone processing (requires PLY file input)
python3 scripts/processing.py

# Debug version with additional logging
python3 scripts/processing_debug.py
```

## Topics

### Published Topics
- `/semantic_map_meshes` (visualization_msgs/MarkerArray): Semantic mesh visualization

### Subscribed Topics
- `/head_front_camera/rgb/image_raw` (sensor_msgs/Image): RGB camera stream
- `/head_front_camera/depth/image_raw` (sensor_msgs/Image): Depth camera stream
- `/head_front_camera/rgb/camera_info` (sensor_msgs/CameraInfo): Camera intrinsics

## Configuration

Edit the scripts to modify:
- YOLO model path (currently: `yolov8n-seg.pt`)
- Detection classes (currently: 56=chair, 60=dining table)
- Mesh parameters (alpha, simplification triangles)
- File paths for mesh output

## Troubleshooting

1. **"No module named 'rclpy'"**: Ensure ROS2 environment is sourced:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **"No module named 'open3d'"**: Install with pip:
   ```bash
   pip install open3d
   ```

3. **"Could not find mesh files"**: Check file paths in semantic_publisher.py (currently hardcoded to `/ros2_ws/`)

4. **No camera topics**: Ensure the camera driver/simulator is running

## File Structure

```
mesh_semantic_mapping/
├── CMakeLists.txt
├── package.xml
├── mesh_semantic_mapping/
│   └── __init__.py
├── scripts/
│   ├── semantic_publisher.py      # Main ROS2 node for mesh visualization
│   ├── test_yolo.py              # YOLO detection test node
│   ├── vision_snapshot.py        # RGB-D capture and processing node
│   ├── processing.py             # Point cloud to mesh conversion
│   ├── processing_debug.py       # Debug version of processing
│   └── yolov8n-seg.pt           # YOLO model weights
└── launch/
    ├── semantic_publisher.launch.py
    ├── test_yolo.launch.py
    └── vision_snapshot.launch.py
```

## Notes

- All scripts use Portuguese logging messages
- Mesh files are expected at `/ros2_ws/table_mesh.stl` and `/ros2_ws/chair_mesh.stl`
- YOLO model is loaded from `yolov8n-seg.pt` in the scripts directory
- The package uses ament_cmake_python for Python-based ROS2 nodes
=======
# Mesh-Semantic-Mapping
>>>>>>> 6c1fa09396a2d95e17cd8b14c271f81a02b073ae
