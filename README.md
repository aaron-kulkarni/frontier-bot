# TurtleBot3 AutoNav: Autonomous Exploration & Mapping with ROS 2

## Overview

FrontierBot is my implementation of a basic autonomous exploration, mapping, and navigation bot using a TurtleBot3 burger model. It integrates standard ROS 2 tools(slam_toolbox, Nav2, Gazebo, and Rviz2) and implements a frontier exploration algorithm for mapping unknown environments.

---

## Key Features

- **Autonomous Frontier-Based Exploration:**
  Implements a custom frontier exploration algorithm to autonomously map unknown environments. The robot identifies frontiers (boundaries between known and unknown space), clusters them, and navigates to the most promising locations to maximize map coverage with minimal human intervention.

- **Real-Time SLAM:**
  Integrates `slam_toolbox` for robust, real-time mapping and localization.

- **Autonomous Navigation:**
  Utilizes the Nav2 stack for dynamic path planning, obstacle avoidance, and goal execution.

- **Simulation-Ready:**
  Launches a complete TurtleBot3 (burger model) simulation in Gazebo, including physics, sensors, and realistic environments.
---

## Frontier Exploration Algorithm

At the heart of this project is a custom **frontier exploration algorithm** that enables the robot to autonomously explore and map unfamiliar terrain:

1. **Frontier Detection:**
   The algorithm continuously analyzes the occupancy grid to identify *frontier cells*—locations on the boundary between explored (known) and unexplored (unknown) space.

2. **Clustering:**
   Detected frontier cells are grouped into clusters using a region-growing approach. Each cluster represents a contiguous area of unexplored boundary.

3. **Centroid Selection:**
   For each cluster, the centroid is computed. The robot selects the closest non-blacklisted centroid as its next exploration goal, prioritizing efficient coverage.

4. **Goal Validation & Navigation:**
   The algorithm ensures the selected goal is in free space (or finds the nearest free cell), then dispatches a navigation goal via the Nav2 action server.

5. **Failure Handling:**
   If a goal is unreachable, it is blacklisted to prevent repeated attempts, ensuring robust and efficient exploration.

This approach enables the robot to autonomously build a complete map of an unknown environment, adapting in real time to obstacles and navigation failures.

---

## Prerequisites

- **Ubuntu 24.04 (Noble)**
- **ROS 2 Kilted** (fully installed and sourced)
- **Gazebo Sim 9** and TurtleBot3 simulation packages:
  ```bash
  sudo apt install ros-kilted-turtlebot3-gazebo ros-kilted-gz-sim-vendor ros-kilted-ros-gz-sim
  ```
---

## Usage

To build and launch the full simulation, mapping, and autonomous exploration stack:

```bash
./build.sh
./run.sh
```

- The robot will autonomously explore and map the environment.
- Monitor progress and interact via RViz2.

---

## Technical Highlights

- **Languages:** C++, Python
- **Core ROS 2 Concepts:** Actions, Topics, Timers, Custom Nodes
- **Algorithmic Techniques:** Occupancy grid analysis, BFS-based clustering, centroid calculation, dynamic goal selection, blacklist management
- **Simulation Integration:** Seamless Gazebo and RViz2 orchestration

---

## Credits

- [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [ROS 2 Kilted](https://docs.ros.org/en/kilted/index.html)
- [Navigation2](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
---
