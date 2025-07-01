#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/kilted/setup.bash

# Build with compile_commands.json for clangd
# colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon build --symlink-install --packages-select turtlebot3_autonav --cmake-clean-cache

# Symlink compile_commands.json for clangd (optional, but helpful)
ln -sf build/turtlebot3_autonav/compile_commands.json compile_commands.json

echo "Build complete and workspace sourced!"
