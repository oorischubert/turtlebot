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
        os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a2m8_launch.py')
    ),
    launch_arguments={
        'scan_mode': 'Boost',
        'frame_id': 'base_link'  # Set the frame_id for the RPLIDAR
    }.items()
)

    static_tf_broadcaster = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],  # Adjust these values as needed
    name='static_tf_broadcaster'
)


    return launch.LaunchDescription([
        static_tf_broadcaster,
        motor_controller,
        rplidar_launch,
    ])

    #rplidar: 
    #git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
    #ros2 launch rplidar_ros rplidar_a2m8_launch.py frame_id:=base_link
    
    #base_link: 
    #sudo apt install ros-humble-tf2-ros
    #ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link

    # usb port connections:
    # |------------------|
    # | rplidar | esp32  |
    # |------------------|
    # | empty   | empty  |
    # |------------------|
