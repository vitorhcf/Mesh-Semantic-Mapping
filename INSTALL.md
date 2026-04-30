# Installation Guide - mesh_semantic_mapping

This guide provides step-by-step instructions to set up and run the `mesh_semantic_mapping` ROS2 package.

## Prerequisites

- Ubuntu 22.04 LTS (or compatible)
- ROS2 Humble installed
- Python 3.10+
- Git

## Installation Steps

### 1. Verify ROS2 Installation

```bash
# Check ROS2 is installed
which ros2
ros2 --version

# Source ROS2 environment
source /opt/ros/humble/setup.bash
```

### 2. Install System Dependencies

```bash
sudo apt-get update
sudo apt-get install -y \
  python3-pip \
  libopencv-dev \
  python3-opencv \
  ros-humble-sensor-msgs \
  ros-humble-visualization-msgs \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-message-filters \
  ros-humble-tf2-ros \
  ros-humble-ament-cmake-python
```

### 3. Install Python Dependencies

```bash
pip install --upgrade pip
pip install \
  open3d \
  ultralytics \
  numpy \
  opencv-python \
  opencv-contrib-python
```

### 4. Build the Package

Navigate to your ROS2 workspace and build:

```bash
cd /mnt/ssd/pcl/ros2_ws

# Build only mesh_semantic_mapping
colcon build --packages-select mesh_semantic_mapping

# Or build with verbose output for debugging
colcon build --packages-select mesh_semantic_mapping --event-handlers console_direct+

# Build entire workspace
colcon build
```

### 5. Source the Workspace

After successful build:

```bash
source /mnt/ssd/pcl/ros2_ws/install/setup.bash
```

Add to your `.bashrc` for automatic sourcing:

```bash
echo "source /mnt/ssd/pcl/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 6. Verify Installation

```bash
# Check package is installed
ros2 pkg list | grep mesh_semantic_mapping

# List available executables
ros2 pkg executables mesh_semantic_mapping

# List launch files
ros2 launch mesh_semantic_mapping --help
```

## Verification Commands

### Test Individual Nodes (without camera)

```bash
# Test that semantic_publisher can load (will wait for mesh files)
timeout 5 ros2 run mesh_semantic_mapping semantic_publisher.py || true

# Check Python imports work
python3 -c "import rclpy; import cv2; import open3d; import ultralytics; print('All imports OK!')"
```

### Test with Simulator/Camera

1. **Start your robot simulator or camera node:**
   ```bash
   # For TIAGo simulator
   ros2 launch tiago_gazebo tiago.launch.py
   ```

2. **In another terminal, run the vision node:**
   ```bash
   ros2 launch mesh_semantic_mapping vision_snapshot.launch.py
   ```

3. **Check for output:**
   ```bash
   # Should see detection messages
   # Check for generated images
   ls -lh visao_do_robo_segmentada.jpg
   ```

## File Structure

```
/mnt/ssd/pcl/ros2_ws/src/mesh_semantic_mapping/
├── CMakeLists.txt                    # Build configuration
├── package.xml                       # Package metadata
├── setup.py                          # Python package setup
├── README.md                         # Package documentation
├── INSTALL.md                        # This file
├── QUICKSTART.py                     # Quick start guide
├── config.yaml                       # Configuration template
├── mesh_semantic_mapping/
│   └── __init__.py                   # Python package init
├── scripts/
│   ├── semantic_publisher.py         # Main ROS2 node (mesh visualization)
│   ├── test_yolo.py                 # YOLO detection test node
│   ├── vision_snapshot.py           # RGB-D capture and processing node
│   ├── processing.py                # Point cloud to mesh conversion
│   ├── processing_debug.py          # Debug version of processing
│   └── yolov8n-seg.pt               # YOLO model weights (7MB)
├── launch/
│   ├── semantic_publisher.launch.py
│   ├── test_yolo.launch.py
│   └── vision_snapshot.launch.py
└── resource/
    └── mesh_semantic_mapping         # ROS2 package resource marker
```

## Troubleshooting

### Build Issues

**Problem:** `CMake Error: Could not find ament_cmake_python`
```bash
# Solution: Install build dependency
sudo apt-get install ros-humble-ament-cmake-python
```

**Problem:** `Package 'cv_bridge' not found`
```bash
# Solution: Install the dependency
sudo apt-get install ros-humble-cv-bridge
```

**Problem:** Build takes too long or fails
```bash
# Clean and rebuild
cd /mnt/ssd/pcl/ros2_ws
rm -rf build install log
colcon build --packages-select mesh_semantic_mapping
```

### Runtime Issues

**Problem:** `ModuleNotFoundError: No module named 'open3d'`
```bash
# Solution: Install with pip
pip install open3d

# Or with conda (if using conda)
conda install -c conda-forge open3d
```

**Problem:** `Failed to subscribe to /head_front_camera/rgb/image_raw`
```bash
# Solution: Ensure camera driver/simulator is running
# Check available topics
ros2 topic list | grep camera

# If no camera topics, start simulator:
ros2 launch tiago_gazebo tiago.launch.py
```

**Problem:** `FileNotFoundError: table_mesh.stl not found`
```bash
# Solution: Generate mesh first using processing script
cd /path/to/point_clouds
python3 scripts/processing.py

# Or create test mesh files at expected location
mkdir -p /ros2_ws
# Copy or generate mesh files there
```

**Problem:** YOLO detection is slow or crashes
```bash
# Solution: Use a lighter model or GPU acceleration
# Edit scripts to use different model:
# self.yolo_model = YOLO('yolov8n-seg.pt')  # nano
# self.yolo_model = YOLO('yolov8s-seg.pt')  # small (faster)

# For GPU support, ensure CUDA is installed
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"
```

### Permission Issues

**Problem:** `Permission denied` when running scripts
```bash
# Solution: Make scripts executable
chmod +x /mnt/ssd/pcl/ros2_ws/src/mesh_semantic_mapping/scripts/*.py
```

## Development Setup

### For Package Development

```bash
# Clone or modify the package
cd /mnt/ssd/pcl/ros2_ws/src/mesh_semantic_mapping

# Install in development mode
pip install -e .

# Test imports
python3 -c "import mesh_semantic_mapping; print('Package imported successfully')"
```

### Running Tests

```bash
# Build with testing enabled
colcon build --packages-select mesh_semantic_mapping --cmake-args -DBUILD_TESTING=ON

# Run tests
colcon test --packages-select mesh_semantic_mapping
```

## Next Steps

1. Read the [README.md](README.md) for detailed package documentation
2. Check the [config.yaml](config.yaml) for configuration options
3. Run the [QUICKSTART.py](QUICKSTART.py) script for guided setup:
   ```bash
   python3 QUICKSTART.py
   ```
4. Test individual nodes with your robot or simulator

## Getting Help

### Common Resources

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [ROS2 Package Documentation](https://index.ros.org/doc/ros2/)
- [Open3D Documentation](http://www.open3d.org/docs/)
- [YOLOv8 Documentation](https://docs.ultralytics.com/)

### Debug Commands

```bash
# Show detailed package information
ros2 pkg prefix mesh_semantic_mapping

# Check Python path
python3 -c "import sys; print('\\n'.join(sys.path))"

# Verify ROS2 environment variables
env | grep ROS

# Check installed packages
pip list | grep -E "open3d|ultralytics|opencv"
```

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review log files: `cat ~/.ros/ros.log` or `ros2 run mesh_semantic_mapping <node> --ros-args --log-level debug`
3. Check ROS2 topics and services: `ros2 topic list`, `ros2 service list`

---

**Last Updated:** 2024
**Package Version:** 0.0.0
