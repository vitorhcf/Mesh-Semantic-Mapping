# Index - mesh_semantic_mapping Package

## 📋 Documentation Index

Start here based on your needs:

### 🚀 Getting Started (First Time)
1. **[QUICKSTART.py](QUICKSTART.py)** - Interactive setup guide (run with `python3 QUICKSTART.py`)
2. **[QUICKREF.md](QUICKREF.md)** - Quick reference card with commands
3. **[README.md](README.md)** - Full package overview and features

### 📦 Installation & Setup
- **[INSTALL.md](INSTALL.md)** - Complete installation guide with prerequisites and troubleshooting
- **[SETUP_SUMMARY.md](SETUP_SUMMARY.md)** - Summary of what was configured for ROS2
- **[config.yaml](config.yaml)** - Configuration template with all parameters

### 🛠️ Build & Run
- **[Makefile](Makefile)** - Build commands (`make help` to see all)
- **CMakeLists.txt** - CMake build configuration
- **package.xml** - ROS2 package metadata and dependencies
- **setup.py** - Python package setup

### 📝 Package Files
```
mesh_semantic_mapping/
├── CMakeLists.txt           ← Build config (updated for Python)
├── package.xml              ← ROS2 metadata (dependencies added)
├── setup.py                 ← Python package setup (NEW)
├── Makefile                 ← Build commands (NEW)
│
├── README.md                ← Full documentation (NEW)
├── INSTALL.md               ← Installation guide (NEW)
├── QUICKREF.md              ← Quick reference (NEW)
├── SETUP_SUMMARY.md         ← Setup summary (NEW)
├── QUICKSTART.py            ← Interactive setup (NEW)
├── INDEX.md                 ← This file (NEW)
├── config.yaml              ← Configuration template (NEW)
│
├── mesh_semantic_mapping/
│   └── __init__.py          ← Python module (NEW)
│
├── launch/                  ← Launch files (NEW)
│   ├── semantic_publisher.launch.py
│   ├── test_yolo.launch.py
│   └── vision_snapshot.launch.py
│
├── scripts/
│   ├── semantic_publisher.py      ← ROS2 node (ready)
│   ├── test_yolo.py               ← ROS2 node (ready)
│   ├── vision_snapshot.py         ← ROS2 node (ready)
│   ├── processing.py              ← Standalone (ready)
│   ├── processing_debug.py        ← Standalone debug (ready)
│   └── yolov8n-seg.pt             ← YOLO model (7MB)
│
└── resource/
    └── mesh_semantic_mapping      ← ROS2 marker (NEW)
```

## ⚡ Quick Commands

### Build
```bash
make build              # Build package
make fullinstall        # Install deps + build
make clean              # Clean build
```

### Run
```bash
make run-publisher      # Run semantic publisher
make run-yolo           # Run YOLO detection
make run-vision         # Run vision snapshot
make run-processing     # Run processing script
```

### Utilities
```bash
make help               # Show all commands
make quickstart         # Interactive setup
make verify             # Verify installation
```

## 🎯 Node Information

### semantic_publisher.py
- **Type**: ROS2 Node
- **Launch**: `ros2 launch mesh_semantic_mapping semantic_publisher.launch.py`
- **Publishes**: `/semantic_map_meshes` (MarkerArray)
- **Purpose**: Visualize semantic meshes in RViz
- **Requires**: `/ros2_ws/table_mesh.stl`, `/ros2_ws/chair_mesh.stl`

### test_yolo.py
- **Type**: ROS2 Node
- **Launch**: `ros2 launch mesh_semantic_mapping test_yolo.launch.py`
- **Subscribes**: `/head_front_camera/rgb/image_raw`
- **Output**: `visao_do_robo_segmentada.jpg`
- **Purpose**: YOLO object detection (tables & chairs)

### vision_snapshot.py
- **Type**: ROS2 Node
- **Launch**: `ros2 launch mesh_semantic_mapping vision_snapshot.launch.py`
- **Subscribes**: RGB-D + CameraInfo (TIAGo camera)
- **Purpose**: Capture and process RGB-D data with semantic segmentation

### processing.py
- **Type**: Standalone Python script
- **Purpose**: Convert PLY point clouds to STL mesh files
- **Input**: PLY file (e.g., `table.ply`)
- **Output**: STL mesh file (e.g., `table_mesh.stl`)

## 📊 What Was Configured

### ROS2 Integration
✅ Updated `package.xml` with:
  - `ament_cmake_python` build tool
  - 7 ROS2 dependencies (rclpy, sensor_msgs, visualization_msgs, cv_bridge, image_transport, message_filters, tf2_ros)

✅ Updated `CMakeLists.txt` with:
  - Python package configuration
  - Script installation (5 Python nodes)
  - Launch files installation
  - YOLO model installation

✅ Created launch files for all 3 ROS2 nodes

✅ Made all scripts executable with proper shebang lines

### Python Package
✅ Created `setup.py` for package installation
✅ Created `mesh_semantic_mapping/__init__.py` (Python module)
✅ Created `resource/mesh_semantic_mapping` (ROS2 marker)

### Documentation
✅ README.md - Full documentation
✅ INSTALL.md - Installation guide with troubleshooting
✅ QUICKREF.md - Quick reference card
✅ SETUP_SUMMARY.md - What was set up
✅ QUICKSTART.py - Interactive setup guide
✅ config.yaml - Configuration template
✅ Makefile - Build commands
✅ INDEX.md - This file

## 🔧 Dependencies

### ROS2 Dependencies (in package.xml)
```
rclpy                    # ROS2 Python client
sensor_msgs              # Camera and sensor messages
visualization_msgs       # RViz visualization messages
cv_bridge                # OpenCV to ROS2 bridge
image_transport          # Image transport utilities
message_filters          # Message synchronization
tf2_ros                  # Transform frame library
ament_cmake_python       # Python CMake support
```

### Python Dependencies
```
open3d                   # Point cloud processing
ultralytics              # YOLO object detection
numpy                    # Numerical computing
opencv-python            # Computer vision
cv_bridge                # (ROS2)
rclpy                    # (ROS2)
```

## 📋 Installation Checklist

- [ ] ROS2 Humble installed
- [ ] System dependencies: `libopencv-dev`, `python3-opencv`
- [ ] ROS2 packages: sensor_msgs, visualization_msgs, cv_bridge, tf2_ros, etc.
- [ ] Python packages: open3d, ultralytics, opencv-python
- [ ] Package built: `colcon build --packages-select mesh_semantic_mapping`
- [ ] Environment sourced: `source install/setup.bash`
- [ ] Scripts executable: `chmod +x scripts/*.py` (already done)

## 🚀 First Time Setup (3 steps)

1. **Build**:
   ```bash
   cd /mnt/ssd/pcl/ros2_ws
   colcon build --packages-select mesh_semantic_mapping
   ```

2. **Source**:
   ```bash
   source install/setup.bash
   ```

3. **Run**:
   ```bash
   ros2 launch mesh_semantic_mapping semantic_publisher.launch.py
   ```

## 📖 Documentation Files

| File | Purpose | Best For |
|------|---------|----------|
| **README.md** | Full documentation | Understanding package features |
| **INSTALL.md** | Installation guide | Setup and troubleshooting |
| **QUICKREF.md** | Quick reference | Remembering commands |
| **QUICKSTART.py** | Interactive guide | First-time setup |
| **SETUP_SUMMARY.md** | Setup summary | Understanding what changed |
| **config.yaml** | Configuration template | Understanding parameters |
| **Makefile** | Build commands | Quick builds and runs |
| **INDEX.md** | This file | Finding documentation |

## 🆘 Getting Help

1. **Can't build?** → See [INSTALL.md](INSTALL.md) troubleshooting section
2. **Don't know how to run?** → Run `python3 QUICKSTART.py` or see [QUICKREF.md](QUICKREF.md)
3. **What parameters can I configure?** → See [config.yaml](config.yaml)
4. **Want to understand the package?** → Read [README.md](README.md)
5. **Need commands?** → See [QUICKREF.md](QUICKREF.md) or run `make help`

## 🔗 Related Topics

### ROS2 Concepts Used
- Python nodes (rclpy)
- Message publishing/subscription
- Launch files
- TF2 transforms
- Camera synchronization

### Technologies
- Point cloud processing (Open3D)
- Object detection (YOLOv8)
- Computer vision (OpenCV)
- ROS2 Humble

## ✨ Key Features

✅ Full ROS2 integration
✅ Ready-to-run launch files
✅ Comprehensive documentation
✅ Interactive setup guide
✅ Makefile for convenient commands
✅ Configuration template
✅ Executable scripts with proper shebangs
✅ Python package structure
✅ Clear troubleshooting guides

## 📍 Package Location

```
/mnt/ssd/pcl/ros2_ws/src/mesh_semantic_mapping/
```

## 🎓 Learning Resources

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Open3D Docs](http://www.open3d.org/docs/)
- [YOLOv8 Docs](https://docs.ultralytics.com/)
- [ROS2 Python Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

---

**Version**: 0.0.0
**ROS2 Distribution**: Humble
**Last Updated**: 2024-04-30
**Status**: ✅ Ready to Build and Run
