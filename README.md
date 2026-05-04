# Mesh Semantic Mapping

A ROS2 package for semantic mesh mapping and robot perception using object segmentation and mesh reconstruction.

## Features

- **Semantic Publisher**: Publishes semantic mesh markers for visualization in RViz
- **YOLO Object Detection**: Detects tables and chairs in camera images
- **Vision Snapshot**: Captures and processes depth+RGB data with semantic segmentation
- **Mesh Processing**: Converts point clouds to STL mesh files

## Running the pipeline

Runs the pipeline sequentially using only one launch file (in development):
```bash
ros2 launch mesh_semantic_mapping pipeline.launch.py
```

## Running the Nodes

### 1. Vision Snapshot

Captures synchronized RGB-D data and performs semantic segmentation:

```bash
ros2 run mesh_semantic_mapping vision_snapshot.py
```

Subscribes to:
- `/head_front_camera/rgb/image_raw`
- `/head_front_camera/depth/image_raw`
- `/head_front_camera/rgb/camera_info`

### 2. Processing Scripts

Process point clouds to generate meshes:

```bash
# Standalone processing (requires PLY file input)
python3 scripts/processing.py

# Debug version with additional visualization
python3 scripts/processing_debug.py
```

### 3. Semantic Publisher

Publishes mesh markers to `/semantic_map_meshes` topic:

```bash
ros2 run mesh_semantic_mapping semantic_publisher.py
```

Expected files:
- `/ros2_ws/table_mesh.stl`
- `/ros2_ws/chair_mesh.stl`

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

