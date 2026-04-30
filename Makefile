.PHONY: help build clean install run-publisher run-yolo run-vision run-processing test source colcon-info debug

WS_ROOT := /mnt/ssd/pcl/ros2_ws
PKG_NAME := mesh_semantic_mapping

help:
	@echo "==========================================="
	@echo "  mesh_semantic_mapping - Available Commands"
	@echo "==========================================="
	@echo ""
	@echo "Build & Setup:"
	@echo "  make build              - Build the package with colcon"
	@echo "  make clean              - Clean build artifacts"
	@echo "  make install            - Install Python dependencies"
	@echo "  make source             - Source the ROS2 environment"
	@echo ""
	@echo "Run Nodes:"
	@echo "  make run-publisher      - Run semantic publisher node"
	@echo "  make run-yolo           - Run YOLO detection node"
	@echo "  make run-vision         - Run vision snapshot node"
	@echo "  make run-processing     - Run standalone processing script"
	@echo ""
	@echo "Utilities:"
	@echo "  make test               - Run package tests"
	@echo "  make colcon-info        - Show colcon package info"
	@echo "  make debug              - Run with debug output"
	@echo "  make quickstart         - Run interactive setup guide"
	@echo ""

build:
	@echo "Building package: $(PKG_NAME)"
	cd $(WS_ROOT) && colcon build --packages-select $(PKG_NAME)
	@echo "✅ Build complete. Run 'source $(WS_ROOT)/install/setup.bash'"

clean:
	@echo "Cleaning build artifacts..."
	cd $(WS_ROOT) && rm -rf build install log
	@echo "✅ Clean complete"

install-deps:
	@echo "Installing Python dependencies..."
	pip install open3d ultralytics numpy opencv-python opencv-contrib-python
	@echo "✅ Dependencies installed"

install-ros-deps:
	@echo "Installing ROS2 dependencies..."
	sudo apt-get install -y \
		ros-humble-sensor-msgs \
		ros-humble-visualization-msgs \
		ros-humble-cv-bridge \
		ros-humble-image-transport \
		ros-humble-message-filters \
		ros-humble-tf2-ros \
		ros-humble-ament-cmake-python
	@echo "✅ ROS2 dependencies installed"

source:
	@echo "Don't forget to source in your current shell!"
	@echo "Run: source $(WS_ROOT)/install/setup.bash"

run-publisher:
	@echo "Running semantic publisher node..."
	ros2 launch $(PKG_NAME) semantic_publisher.launch.py

run-yolo:
	@echo "Running YOLO detection node..."
	ros2 launch $(PKG_NAME) test_yolo.launch.py

run-vision:
	@echo "Running vision snapshot node..."
	ros2 launch $(PKG_NAME) vision_snapshot.launch.py

run-processing:
	@echo "Running standalone processing script..."
	python3 scripts/processing.py

test:
	@echo "Running tests..."
	cd $(WS_ROOT) && colcon test --packages-select $(PKG_NAME) --event-handlers console_direct+

colcon-info:
	@echo "Package information:"
	@ros2 pkg prefix $(PKG_NAME) 2>/dev/null || echo "Package not found. Build first with: make build"
	@echo ""
	@echo "Available executables:"
	@ros2 pkg executables $(PKG_NAME) 2>/dev/null || echo "Run: source install/setup.bash"

debug:
	@echo "Running with debug output..."
	ROS_LOG_DIR=$(WS_ROOT)/log ros2 run $(PKG_NAME) semantic_publisher.py --ros-args --log-level debug

quickstart:
	@echo "Running interactive setup guide..."
	python3 $(WS_ROOT)/src/$(PKG_NAME)/QUICKSTART.py

verify:
	@echo "Verifying installation..."
	@echo "✓ Checking ROS2..."
	@which ros2 > /dev/null && echo "  ROS2 found" || echo "  ROS2 not found"
	@echo "✓ Checking Python packages..."
	@python3 -c "import rclpy; import cv2; import open3d; import ultralytics; print('  All packages OK')" || echo "  Missing packages"
	@echo "✓ Checking package..."
	@ros2 pkg list | grep $(PKG_NAME) > /dev/null && echo "  Package found" || echo "  Package not found"

list-nodes:
	@echo "Available nodes:"
	@echo "  - semantic_publisher.py    (mesh visualization)"
	@echo "  - test_yolo.py             (YOLO detection)"
	@echo "  - vision_snapshot.py       (RGB-D capture)"
	@echo "  - processing.py            (point cloud to mesh)"

list-topics:
	@echo "Published topics:"
	@echo "  - /semantic_map_meshes          (MarkerArray)"
	@echo ""
	@echo "Subscribed topics:"
	@echo "  - /head_front_camera/rgb/image_raw"
	@echo "  - /head_front_camera/depth/image_raw"
	@echo "  - /head_front_camera/rgb/camera_info"

# Advanced targets
rebuild: clean build
	@echo "✅ Clean rebuild complete"

fullinstall: install-deps install-ros-deps build
	@echo "✅ Full installation complete"
	@echo "   Run: source $(WS_ROOT)/install/setup.bash"

check-docs:
	@echo "Documentation files:"
	@ls -lh $(WS_ROOT)/src/$(PKG_NAME)/*.md 2>/dev/null || echo "Markdown files not found"

.DEFAULT_GOAL := help
