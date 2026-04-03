import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'turtlebot'
    pkg_share = get_package_share_directory(package_name)

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'joystick.launch.py')
        )
    )

    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_drive_base_controller/cmd_vel')]
    )

    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rplidar.launch.py')
        )
    )

    delayed_rplidar_launch = TimerAction(period=2.0, actions=[rplidar])

    controller_params_file = os.path.join(pkg_share, 'config', 'my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[controller_params_file],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                    "--controller-manager", "/controller_manager",
                    "--param-file", controller_params_file],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base_controller",
                    "--controller-manager", "/controller_manager",
                    "--param-file", controller_params_file],
    )

    delay_robot_controller_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        delayed_controller_manager,
        delay_robot_controller_after_jsb,
        delayed_joint_broad_spawner,
        delayed_rplidar_launch,
    ])

# ros2 launch turtlebot launch_robot.launch.py
#
# rpi5 usb port connections:
# |------------------|
# | esp32  | rplidar |
# |------------------|
# | empty  |   joy   |
# |------------------|
