#!/bin/bash

echo "========== plain_slam_ros2 total code information =========="
cloc src include/plain_slam include/plain_slam_ros2

echo -e "\n\n"

echo "========== plain_slam total C++ code information (excluding ROS2-related code) =========="
find src/ -name "*.cpp" \
  | grep -v "_node.cpp" \
  | grep -v "ros_utils.cpp" \
  | grep -v "scan_intensity_matcher.cpp" \
  | xargs cloc
