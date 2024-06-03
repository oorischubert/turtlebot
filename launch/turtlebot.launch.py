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
            'frame_id': 'base_link',
            #'use_sim_time': 'true'
        }.items()
    )

    delayed_rplidar_launch = TimerAction(
        period=2.0,  # Delay in seconds before starting the RPLIDAR to prevent startup interference from other processes
        actions=[rplidar_launch]
    )

    #deprecates as i now have odometry data from motor controller
    # static_tf_broadcaster = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
    #     name='static_tf_broadcaster'
    # )

    motor_controller = Node(
        package='turtlebot',
        executable='motor_controller',
        name='motor_controller',
        #parameters=[{'use_sim_time': True}] 
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

    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    return LaunchDescription([
        #static_tf_broadcaster,
        delayed_rplidar_launch,  # Use the delayed action for RPLIDAR
        motor_controller,
        on_motor_controller_exit,
        static_transform_publisher
    ])


    #turtlebot:
    #apt-get install -y python3-pip
    #pip3 install pyserial
    #pip3 install transforms3d
    #git clone https://github.com/oorischubert/turtlebot
    #source install/setup.bash
    #ros2 launch turtlebot turtlebot_launch.py
   
    #local keyboard teleop
    #ros2 run turtlebot teleop

    #rplidar: 
    #git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
    #ros2 launch rplidar_ros rplidar_a2m8_launch.py frame_id:=base_link

    #base_link: 
    #sudo apt install ros-humble-tf2-ros
    #ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link

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

