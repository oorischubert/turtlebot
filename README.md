# TurtleBot ROS2 Project

A ROS2-based TurtleBot simulation and control project that provides a complete environment for robot simulation, navigation, and control for Oori's custom TurtleBot variant.

## Overview

This project implements control and simulation environments for TurtleBot with ROS2, featuring:

- Gazebo simulation integration
- Robot state publishing
- Joystick control
- Twist multiplexer for velocity control
- ROS2-Gazebo bridge for sensor data
- Support for multiple simulation worlds

## Prerequisites

- ROS2 Humble
- Gazebo
- Required ROS2 packages:
  - ros_gz_sim
  - ros_gz_bridge
  - ros_gz_image
  - twist_mux
  - joystick

## Installation

1. Clone the repository:

```bash
git clone https://github.com/oorischubert/turtlebot.git
cd turtlebot
```

2. Install dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the workspace:

```bash
colcon build
source install/setup.bash
```

## Usage

### Basic Simulation

Launch the basic simulation with an empty world:

```bash
ros2 launch turtlebot gazebo.launch.py
```

### Custom Worlds

The project supports various simulation worlds:

1. Warehouse World:

```bash
ros2 launch turtlebot gazebo.launch.py world:=$HOME/ros2_ws/src/aws-robomaker-small-warehouse-world/worlds/no_roof_small_warehouse/no_roof_small_warehouse.world
```

2. House World:

```bash
ros2 launch turtlebot gazebo.launch.py world:=$HOME/ros2_ws/src/aws-robomaker-small-house-world/worlds/small_house.world
```

3. Bookstore World:

```bash
ros2 launch turtlebot gazebo.launch.py world:=$HOME/ros2_ws/src/aws-robomaker-bookstore-world/worlds/bookstore.world
```

### World Installation

To install the custom worlds, clone the following repositories:

```bash
git clone -b ros2 https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git
git clone -b ros2 https://github.com/aws-robotics/aws-robomaker-small-house-world.git
git clone -b ros2 https://github.com/aws-robotics/aws-robomaker-bookstore-world.git
```

## Project Structure

```
turtlebot/
├── config/           # Configuration files
├── description/      # Robot description files
├── launch/          # Launch files
├── maps/            # Navigation maps
├── worlds/          # Simulation worlds
└── turtlebot/       # Python package
```

## Troubleshooting

1. If you encounter Gazebo plugin issues, set:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib/
```

2. To reset Gazebo if it crashes:

```bash
pkill -9 gzserver
```

## Features

- **Robot State Publisher**: Handles robot state visualization
- **Joystick Control**: Teleoperation support
- **Twist Mux**: Velocity command multiplexing
- **ROS2-Gazebo Bridge**: Sensor data bridging
- **Multiple World Support**: Various simulation environments
- **ROS2 Control**: Joint state and differential drive control

