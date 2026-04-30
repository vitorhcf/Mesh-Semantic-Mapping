# Quick Reference - mesh_semantic_mapping

## Installation (5 minutes)

```bash
# 1. Install dependencies
pip install open3d ultralytics numpy opencv-python
sudo apt-get install -y ros-humble-sensor-msgs ros-humble-visualization-msgs ros-humble-cv-bridge ros-humble-image-transport ros-humble-message-filters ros-humble-tf2-ros

# 2. Build
cd /mnt/ssd/pcl/ros2_ws
colcon build --packages-select mesh_semantic_mapping

# 3. Source
source install/setup.bash

# Add to ~/.bashrc to source automatically
echo "source /mnt/ssd/pcl/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Quick Commands

| Task | Command |
|------|---------|
| **Build** | `make build` or `colcon build --packages-select mesh_semantic_mapping` |
| **Run publisher** | `make run-publisher` or `ros2 launch mesh_semantic_mapping semantic_publisher.launch.py` |
| **Run YOLO** | `make run-yolo` or `ros2 launch mesh_semantic_mapping test_yolo.launch.py` |
| **Run vision** | `make run-vision` or `ros2 launch mesh_semantic_mapping vision_snapshot.launch.py` |
| **Process PC** | `make run-processing` or `python3 scripts/processing.py` |
| **Help** | `make help` |
| **Setup guide** | `make quickstart` or `python3 QUICKSTART.py` |
| **Verify install** | `make verify` |

## Nodes

### semantic_publisher.py
- **Topic**: `/semantic_map_meshes`
- **Type**: MarkerArray
- **Requires**: `/ros2_ws/table_mesh.stl`, `/ros2_ws/chair_mesh.stl`
- **Use**: Visualize meshes in RViz

### test_yolo.py
- **Subscribes**: `/head_front_camera/rgb/image_raw`
- **Output**: `visao_do_robo_segmentada.jpg`
- **Detects**: Tables (class 60), Chairs (class 56)

### vision_snapshot.py
- **Subscribes**: RGB + Depth + CameraInfo from TIAGo
- **Features**: Synchronized RGB-D, TF2 transforms
- **Output**: Segmented images and point clouds

### processing.py
- **Input**: PLY point cloud file
- **Output**: STL mesh file
- **Standalone**: No ROS2 required

## File Locations

```
/mnt/ssd/pcl/ros2_ws/src/mesh_semantic_mapping/
├── scripts/           # Python nodes (executable)
├── launch/            # Launch files
├── config.yaml        # Configuration
├── README.md          # Full documentation
├── INSTALL.md         # Install guide
├── SETUP_SUMMARY.md   # What was set up
└── Makefile           # Build commands
```

## Common Tasks

### Visualize meshes
```bash
# Terminal 1
ros2 launch mesh_semantic_mapping semantic_publisher.launch.py

# Terminal 2
rviz2
# Add MarkerArray for /semantic_map_meshes
```

### Test YOLO detection
```bash
# With camera/simulator running
ros2 launch mesh_semantic_mapping test_yolo.launch.py
# Check output: visao_do_robo_segmentada.jpg
```

### Generate mesh from point cloud
```bash
# Copy PLY file to package directory
cp my_pointcloud.ply scripts/
cd scripts/
python3 processing.py  # Modify script for input/output filenames
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `colcon not found` | Install: `sudo apt install python3-colcon-common` |
| `No module named 'rclpy'` | Source ROS2: `source /opt/ros/humble/setup.bash` |
| `ModuleNotFoundError: open3d` | Install: `pip install open3d` |
| `Failed to subscribe` | Start camera/simulator: `ros2 launch tiago_gazebo tiago.launch.py` |
| `CMake error` | Clean: `rm -rf build install log && colcon build` |
| `Mesh files not found` | Create at `/ros2_ws/table_mesh.stl` and `/ros2_ws/chair_mesh.stl` |

## Important Files

| File | Purpose |
|------|---------|
| `package.xml` | ROS2 package metadata + dependencies |
| `CMakeLists.txt` | Build configuration |
| `setup.py` | Python package setup |
| `config.yaml` | Configuration template |
| `README.md` | Full documentation |
| `INSTALL.md` | Installation guide |
| `QUICKSTART.py` | Interactive setup |
| `Makefile` | Build commands |

## ROS2 Topics

### Published
- `/semantic_map_meshes` (visualization_msgs/MarkerArray)

### Subscribed
- `/head_front_camera/rgb/image_raw` (sensor_msgs/Image)
- `/head_front_camera/depth/image_raw` (sensor_msgs/Image)
- `/head_front_camera/rgb/camera_info` (sensor_msgs/CameraInfo)

## Environment Setup

```bash
# One-time setup
export ROS_DISTRO=humble
source /opt/ros/humble/setup.bash
source /mnt/ssd/pcl/ros2_ws/install/setup.bash

# Check setup
echo $ROS_DISTRO
ros2 pkg list | grep mesh_semantic_mapping
```

## Next Steps

1. **Read**: `README.md` (overview), `INSTALL.md` (detailed)
2. **Setup**: `make fullinstall` (full installation)
3. **Build**: `make build`
4. **Test**: `make verify`
5. **Run**: `make run-publisher` or choose another node

## Support

- Full docs: See `README.md`, `INSTALL.md`, `SETUP_SUMMARY.md`
- Interactive guide: Run `make quickstart` or `python3 QUICKSTART.py`
- Configuration: Edit `config.yaml` or script files directly
- Logs: Check `~/.ros/ros.log` or terminal output

---
**Version**: 0.0.0 | **Package**: mesh_semantic_mapping | **ROS2**: Humble
