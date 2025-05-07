cmake_minimum_required(VERSION 3.8)
project(mycobot_stacking_project)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(control_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Add link directories for PCL
link_directories(${PCL_LIBRARY_DIRS})

# Add definitions for PCL
add_definitions(${PCL_DEFINITIONS})

# Cube Detector Node
add_executable(cube_detector_node
  src/cube_detector_node.cpp
)
ament_target_dependencies(cube_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  moveit_msgs
  shape_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  moveit_ros_planning_interface
  pcl_conversions
)
target_link_libraries(cube_detector_node
  ${PCL_LIBRARIES}
)

# Stacking Manager Node
add_executable(stacking_manager_node
  src/stacking_manager_node.cpp
)
ament_target_dependencies(stacking_manager_node
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_msgs
  shape_msgs
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_planning_interface
  control_msgs
)

# Cube Spawner Node
add_executable(cube_spawner_node
  src/cube_spawner_node.cpp
)
ament_target_dependencies(cube_spawner_node
  rclcpp
  geometry_msgs
  moveit_msgs
  shape_msgs
  moveit_ros_planning_interface
)

# Install executables
install(TARGETS
  cube_detector_node
  stacking_manager_node
  cube_spawner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install launch, worlds, models, rviz, config, and scripts directories
install(DIRECTORY
  launch
  worlds
  models
  rviz
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
