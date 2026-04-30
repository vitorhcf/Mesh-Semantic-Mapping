cmake_minimum_required(VERSION 3.8)
project(mesh_semantic_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)

# Install Python modules
ament_python_install_package(mesh_semantic_mapping)

# Install Python executables
install(PROGRAMS
  scripts/semantic_publisher.py
  scripts/test_yolo.py
  scripts/vision_snapshot.py
  scripts/processing.py
  scripts/processing_debug.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install scripts directory for supporting files like the YOLO model
install(DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}/scripts
  FILES_MATCHING PATTERN "*.pt"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
