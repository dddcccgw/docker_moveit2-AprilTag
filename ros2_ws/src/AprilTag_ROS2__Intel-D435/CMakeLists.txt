cmake_minimum_required(VERSION 3.8)
project(apriltag_ros2_intel_d435)

# Find dependencies
find_package(ament_cmake REQUIRED)

# This is a metapackage - no source code, just groups other packages
# Metapackages in ROS2 only need minimal CMakeLists.txt

ament_package()

