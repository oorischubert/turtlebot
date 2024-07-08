from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, EmitEvent, LogInfo, OpaqueFunction 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
import os
import subprocess

def generate_launch_description():
    rplidar_ros_share = get_package_share_directory('rplidar_ros')
    imu_ros_share = get_package_share_directory('ros-imu-bno055')

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(rplidar_ros_share, 'launch', 'rplidar_a2m8_launch.py')
        ]),
        launch_arguments={
            'scan_mode': 'Boost',
            'frame_id': 'base_link',
            # 'use_sim_time': 'true'
        }.items()
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(imu_ros_share, 'launch', 'imu.launch')
        ])
    )

    delayed_rplidar_launch = TimerAction(
        period=2.0,  # Delay in seconds before starting the RPLIDAR to prevent startup interference from other processes
        actions=[rplidar_launch]
    )

    motor_controller = Node(
        package='turtlebot',
        executable='motor_controller',
        name='motor_controller',
        # parameters=[{'use_sim_time': True}]
    )

    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    # Add the low_level_bridge node from turtlebot_motorBridge package
    low_level_bridge_node = Node(
        package='turtlebot_motorBridge',
        executable='low_level_bridge',
        name='low_level_bridge'
    )

    return LaunchDescription([
        delayed_rplidar_launch,  # Use the delayed action for RPLIDAR
        #motor_controller,
        static_transform_publisher,
        low_level_bridge_node,  # Add the low_level_bridge node here
        #imu_launch  # Include the IMU launch file
    ])

# Additional information for installation and usage

# turtlebot:
# apt-get install -y python3-pip
# pip3 install pyserial
# pip3 install transforms3d
# git clone https://github.com/oorischubert/turtlebot
# source install/setup.bash
# ros2 launch turtlebot turtlebot_launch.py

#turtlebot_motorBridge: 
#git clone https://github.com/oorischubert/turtlebot_motorBridge.git
#ros2 run turtlebot_motorBridge low_level_bridge

# local keyboard teleop
# ros2 run turtlebot teleop

# rplidar:
# git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
# ros2 launch rplidar_ros rplidar_a2m8_launch.py frame_id:=base_link

# base_link:
# sudo apt install ros-humble-tf2-ros
# ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link

# rpi4 usb port connections:
# |------------------|
# | esp32  | rplidar |
# |------------------|
# | empty  |  empty  |
# |------------------|

# rpi5 usb port connections:
# |------------------|
# | rplidar | esp32  |
# |------------------|
# | Bootfs  | empty  |
# |------------------|