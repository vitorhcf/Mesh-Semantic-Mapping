# Mesh Semantic Mapping

A ROS2 package for semantic mesh mapping and robot perception using object segmentation and mesh reconstruction.

## Features

- **Semantic Publisher**: Publishes semantic mesh markers for visualization in RViz
- **YOLO Object Detection**: Detects tables and chairs in camera images
- **Vision Snapshot**: Captures and processes depth+RGB data with semantic segmentation
- **Mesh Processing**: Converts point clouds to STL mesh files

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

