cmake_minimum_required(VERSION 3.8)
project(obstacle_course)

# -----------------------------
# Compiler options
# -----------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------
# Dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# MoveIt 2
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

# Geometry / meshes
find_package(geometric_shapes REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

## INCLUDE(S)
include_directories(include)

# -----------------------------
# Executable
# -----------------------------
add_executable(obstacle_course
  src/obstacle_course.cpp
)

ament_target_dependencies(obstacle_course
  rclcpp
  moveit_core
  sensor_msgs
  moveit_ros_planning_interface
  moveit_msgs
  geometric_shapes
  shape_msgs
  geometry_msgs
)

# -----------------------------
# Install
# -----------------------------
install(
  TARGETS obstacle_course
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------
# Export
# -----------------------------
ament_package()

