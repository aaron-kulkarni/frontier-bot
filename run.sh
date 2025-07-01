#!/bin/bash
set -e

# Trap Ctrl+C (SIGINT) and run pkill when the script is interrupted
trap "echo 'Shutting down Gazebo...'; pkill -f gz" SIGINT

source /opt/ros/kilted/setup.bash
source "$(dirname "$0")/install/setup.bash"
ros2 launch turtlebot3_autonav bringup_sim.launch.py > run_debug.log 2>&1
