from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, EmitEvent, LogInfo, OpaqueFunction 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
import os
import subprocess

def exit_process_function(_launch_context):
    #not working yet!
    cmd = [
        'ros2', 'topic', 'pub', '-t', '1', '/cmd_vel', 'geometry_msgs/msg/Twist',
        '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
    ]
    subprocess.run(cmd, shell=True)

def generate_launch_description():
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a2m8_launch.py')
        ]),
        launch_arguments={
            'scan_mode': 'Boost',
            'frame_id': 'base_link'
        }.items()
    )

    delayed_rplidar_launch = TimerAction(
        period=2.0,  # Delay in seconds before starting the RPLIDAR
        actions=[rplidar_launch]
    )

    static_tf_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        name='static_tf_broadcaster'
    )

    motor_controller = Node(
        package='turtlebot',
        executable='motor_controller',
        name='motor_controller'
    )

    on_motor_controller_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=motor_controller,
            on_exit=[
                LogInfo(msg="Motor shutdown initiated..."),
                EmitEvent(event=Shutdown(reason='Motor controller exited')),
                OpaqueFunction(
                        function=exit_process_function
                    ),
            ]
        )
    )

    return LaunchDescription([
        static_tf_broadcaster,
        delayed_rplidar_launch,  # Use the delayed action for RPLIDAR
        motor_controller,
        on_motor_controller_exit
    ])


    #turtlebot:
    #apt-get install -y python3-pip
    #git clone https://github.com/oorischubert/turtlebot
    #source install/setup.bash
    #ros2 launch turtlebot turtlebot_launch.py
    #ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p key_timeout:=0.1 -r cmd_vel:=/your_custom_topic

    #rplidar: 
    #git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
    #ros2 launch rplidar_ros rplidar_a2m8_launch.py frame_id:=base_link

    #base_link: 
    #sudo apt install ros-humble-tf2-ros
    #ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link

    # usb port connections:
    # |------------------|
    # | empty  |  esp32  |
    # |------------------|
    # | Bootfs | rplidar |
    # |------------------|