#!/bin/bash
set -e

# Clean up AMENT_PREFIX_PATH and CMAKE_PREFIX_PATH of non-existent turtlebot3* paths
export AMENT_PREFIX_PATH=$(echo $AMENT_PREFIX_PATH | tr ':' '\n' | grep -v 'slam_ws/install/turtlebot3' | grep -v 'slam_ws/install/turtlebot3_' | paste -sd: -)
export CMAKE_PREFIX_PATH=$(echo $CMAKE_PREFIX_PATH | tr ':' '\n' | grep -v 'slam_ws/install/turtlebot3' | grep -v 'slam_ws/install/turtlebot3_' | paste -sd: -)

# Source ROS 2
source /opt/ros/kilted/setup.bash

# Build all packages (C++ and Python) in the workspace
colcon build --symlink-install --cmake-clean-cache

# Symlink compile_commands.json for clangd (optional, but helpful)
ln -sf build/turtlebot3_autonav/compile_commands.json compile_commands.json

echo "Build complete and workspace sourced!"
