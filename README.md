# TurtleBot ROS 2 Project

A custom differential-drive TurtleBot built with ROS 2 Jazzy and Gazebo Harmonic. Supports both simulation and real hardware (ESP32 motor controller, RPLidar, USB camera). Features autonomous navigation with Nav2, real-time SLAM mapping, and EKF sensor fusion.

## Features

- **Navigation**: Nav2 stack with DWB local planner, NavfnPlanner global planner, and behavior tree recovery
- **SLAM**: Real-time map building with SLAM Toolbox while navigating
- **Sensor Fusion**: EKF fuses wheel odometry for smooth `odom → base_link` transforms
- **Gazebo Simulation**: Full simulation with Gazebo Harmonic via `gz_ros2_control`
- **Real Robot**: ESP32 serial interface with RPLidar and joystick teleoperation
- **Joystick Control**: Priority-based twist mux (joystick always overrides navigation)
- **TwistStamped**: Full stamped velocity pipeline (Jazzy standard)

## Prerequisites

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic

Install required packages:

```bash
sudo apt install ros-jazzy-gz-harmonic \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-common \
    ros-jazzy-nav2-rviz-plugins \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    ros-jazzy-twist-mux \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-joy \
    ros-jazzy-diff-drive-controller \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-xacro
```

## Installation

```bash
cd ~/ros2_ws/src
git clone -b jazzy https://github.com/oorischubert/turtlebot.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Usage

### Simulation

```bash
# Terminal 1: Launch robot in Gazebo
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib/
ros2 launch turtlebot gazebo.launch.py

# Terminal 2: Launch navigation (EKF + SLAM + Nav2)
ros2 launch turtlebot navigation_full.launch.py use_sim_time:=true

# Terminal 3: Open RViz
ros2 launch turtlebot rviz.launch.py
```

Use a custom world:

```bash
ros2 launch turtlebot gazebo.launch.py world:=/path/to/your/world.sdf
```

### Real Robot

```bash
# Terminal 1: Launch hardware drivers
ros2 launch turtlebot launch_robot.launch.py

# Terminal 2: Launch navigation
ros2 launch turtlebot navigation_full.launch.py
```

Drive with a joystick (hold button 6 to enable, button 7 for turbo).

### Individual Components

Each part of the stack can also be launched separately:

```bash
ros2 launch turtlebot ekf_localization.launch.py   # EKF only
ros2 launch turtlebot slam.launch.py                # SLAM only
ros2 launch turtlebot nav2.launch.py                # Nav2 only
```

## Project Structure

```
turtlebot/
├── config/
│   ├── ekf.yaml                          # EKF sensor fusion (wheel odometry)
│   ├── mapper_params_online_async.yaml   # SLAM Toolbox parameters
│   ├── my_controllers.yaml               # Diff drive controller config
│   ├── nav2_params.yaml                  # Nav2 navigation parameters
│   ├── twist_mux.yaml                    # Velocity command priorities
│   ├── joystick.yaml                     # Joystick mapping
│   ├── gz_bridge.yaml                    # Gazebo-ROS bridge topics
│   └── *.rviz                            # RViz configurations
├── description/                          # Robot URDF/Xacro
│   ├── robot.urdf.xacro                  # Main robot description
│   ├── ros2_control.xacro                # Hardware interfaces
│   └── gazebo_control.xacro              # Gazebo plugins
├── launch/
│   ├── navigation_full.launch.py         # EKF + SLAM + Nav2 (main entry)
│   ├── gazebo.launch.py                  # Simulation startup
│   ├── launch_robot.launch.py            # Real robot startup
│   ├── ekf_localization.launch.py        # EKF sensor fusion
│   ├── slam.launch.py                    # SLAM mapping
│   ├── nav2.launch.py                    # Navigation stack
│   ├── joystick.launch.py                # Joystick teleoperation
│   └── rviz.launch.py                    # Visualization
├── maps/                                 # Saved SLAM maps
└── worlds/                               # Gazebo SDF worlds
```

## TF Tree

```
map ──(SLAM Toolbox)──> odom ──(EKF)──> base_link
```

- **SLAM Toolbox** publishes `map → odom` (localization in the map)
- **EKF** publishes `odom → base_link` (fused wheel odometry)

## Robot Specs

| Parameter | Value |
|-----------|-------|
| Drive | Differential (2 wheels + caster) |
| Radius | 0.085 m |
| Max linear velocity | 0.26 m/s |
| Max angular velocity | 2.0 rad/s |
| LiDAR | 360°, 0.3–12 m range |
| Motor controller | ESP32 at 115200 baud |
| Encoder resolution | 410 counts/rev |
| Control loop | 30 Hz |

## Troubleshooting

If Gazebo can't find plugins:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib/
```

If Gazebo is stuck from a previous run:

```bash
pkill -9 gz
```

## Branches

- `jazzy` — ROS 2 Jazzy + Gazebo Harmonic (active development)
- `humble` — ROS 2 Humble (archived)
