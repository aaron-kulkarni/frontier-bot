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

### 1. Source ROS 2 Kilted and your workspace

```bash
source /opt/ros/kilted/setup.bash
source ~/programming/slam_ws/install/setup.bash
```

### 2. Launch the full simulation, SLAM, and navigation stack

```bash
ros2 launch turtlebot3_autonav bringup_sim.launch.py
```

This will:
- Start Gazebo with a TurtleBot3 (burger) in an empty world
- Start SLAM Toolbox for mapping
- Start RViz2 for visualization and goal setting
- Start the Nav2 navigation stack (after a short delay for SLAM initialization)

---

## Features

- **Live Mapping:**  
  Watch the map build in RViz2 as the robot explores.

- **Autonomous Navigation:**  
  Use the "2D Nav Goal" tool in RViz2 to send navigation goals. The robot will plan and follow a path, avoiding obstacles.

- **Customizable:**  
  - Change the world file by editing the `world` argument in the launch file.
  - Change the robot model (`burger`, `waffle`, `waffle_pi`) with the `turtlebot3_model` launch argument.

---

## Bonus: Saving and Replaying Maps

### Save a Map

After mapping, save the generated map with:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

### Replay Navigation on a Saved Map

1. Modify the launch file to use `amcl` (localization) instead of SLAM.
2. Load your saved map with Nav2.

---

## File Structure

```
turtlebot3_autonav/
├── launch/
│   └── bringup_sim.launch.py   # Main launch file
├── config/                     # (For future parameter YAMLs)
├── maps/                       # (For saved maps)
├── scripts/                    # (For custom scripts)
├── README.md                   # This file
└── LICENSE                     # MIT License
```

---

## Troubleshooting

- **Build errors referencing another ROS distro:**  
  Make sure you only source Kilted, not any other ROS 2 distribution.
- **Simulation not starting:**  
  Ensure all required packages are installed and Gazebo Sim 9 is available.
- **RViz2 or Nav2 not launching:**  
  Wait a few seconds; Nav2 starts after SLAM is initialized.

---

## Credits

- [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [ROS 2 Kilted](https://docs.ros.org/en/kilted/index.html)
- [Navigation2](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)

---

MIT License © 2024 Aaron Kulkarni