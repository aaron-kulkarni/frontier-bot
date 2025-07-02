#!/bin/bash
set -e

echo "Cleaning and rebuilding workspace..."

# Remove build and install directories
echo "Removing build and install directories..."
rm -rf build install log

# Make sure we're in the ROS2 environment
source /opt/ros/kilted/setup.bash

# Rebuild all packages
echo "Building packages..."
colcon build --symlink-install --cmake-clean-cache

# Source the workspace
source install/setup.bash

echo "Clean build complete!"
echo "You can now run the system using ./run.sh"
