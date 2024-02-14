import os
import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    motor_controller = Node(
            package='turtlebot',
            executable='motor_controller',
            name='motor_controller'
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')
        ),
        launch_arguments={
            'scan_mode': 'Boost'
        }.items()
    )

    return launch.LaunchDescription([
        motor_controller,
        rplidar_launch,
    ])

    # usb port connections:
    # |------------------|
    # | rplidar | esp32  |
    # |------------------|
    # | empty   | empty  |
    # |------------------|
