# TurtleBot3 AutoNav
Autonomous TurtleBot3 Simulation, Mapping, and Navigation Stack for ROS 2 Kilted

---

## Overview

This package provides a single launch file to bring up a complete simulated TurtleBot3 (burger model) in Gazebo, perform SLAM (mapping) with `slam_toolbox`, and enable autonomous navigation with the Nav2 stack. RViz2 is also launched for visualization and interactive goal setting.

- **Simulation:** Gazebo Sim 9 with TurtleBot3 in an empty world
- **Mapping:** Real-time SLAM using `slam_toolbox`
- **Navigation:** Nav2 stack for autonomous path planning and obstacle avoidance
- **Visualization:** RViz2 with live map and goal setting

---

## Prerequisites

- **Ubuntu 24.04 (Noble)**
- **ROS 2 Kilted** (fully installed and sourced)
- **Gazebo Sim 9** and TurtleBot3 simulation packages:
  ```
  sudo apt install ros-kilted-turtlebot3-gazebo ros-kilted-gz-sim-vendor ros-kilted-ros-gz-sim
  ```
- **Workspace built:**
  ```
  cd ~/programming/slam_ws
  colcon build --symlink-install
  ```

---

## Usage

./build.sh
./run.sh

---
## Credits

- [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [ROS 2 Kilted](https://docs.ros.org/en/kilted/index.html)
- [Navigation2](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)

---
