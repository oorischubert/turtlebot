import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            RegisterEventHandler)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    package_name = 'turtlebot'
    pkg_share = get_package_share_directory(package_name)

    default_world = os.path.join(pkg_share, 'worlds', 'empty.world')
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'joystick.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': 'true'
        }.items()
    )

    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_drive_base_controller/cmd_vel')]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
            )
        ]),
        launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'my_bot',
            '-topic', '/robot_description',
            '-z', '0.1',
        ],
    )

    controller_params_file = os.path.join(pkg_share, 'config', 'my_controllers.yaml')

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                    "--param-file", controller_params_file],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base_controller",
                    "--param-file", controller_params_file],
    )

    delay_diff_drive_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    bridge_params = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    return LaunchDescription([
        world_arg,
        rsp,
        joystick,
        twist_mux,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_spawner,
        delay_diff_drive_after_jsb,
        ros_gz_bridge,
        ros_gz_image_bridge,
    ])

# export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib/
#
# ros2 launch turtlebot gazebo.launch.py world:=$HOME/ros2_ws/src/aws-robomaker-small-warehouse-world/worlds/no_roof_small_warehouse/no_roof_small_warehouse.world
# ros2 launch turtlebot gazebo.launch.py world:=$HOME/ros2_ws/src/aws-robomaker-small-house-world/worlds/small_house.world
# ros2 launch turtlebot gazebo.launch.py world:=$HOME/ros2_ws/src/aws-robomaker-bookstore-world/worlds/bookstore.world
