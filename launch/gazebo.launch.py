import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            RegisterEventHandler, ExecuteProcess)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.actions import TimerAction

def generate_launch_description():

    # Set your package name
    package_name = 'turtlebot'

    # Define the default world file path
    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
    )
    
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )

      # Include the joystick launch file
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name), 'launch', 'joystick.launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )


    # Include the robot_state_publisher launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': 'true'
        }.items()
    )

    # Define the twist_mux node
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_drive_base_controller/cmd_vel_unstamped')]
    )

    # Include the Gazebo launch file from ros_gz_sim
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

    joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_base_controller'],
        output='screen'
    )

    # Define the bridge nodes
    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
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

    # Launch all components
    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        world_arg,
        gazebo,
        spawn_entity,
        joint_state_controller,
        diff_drive_controller,
        ros_gz_bridge,
        ros_gz_image_bridge
    ])

# export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib/
# ps aux | grep gzserver   <- reset background segmentation fault

#warehouse world launch:
# ros2 launch turtlebot gazebo.launch.py world:=$HOME/ros2_ws/src/aws-robomaker-small-warehouse-world/worlds/no_roof_small_warehouse/no_roof_small_warehouse.world
#house world launch:
# ros2 launch turtlebot gazebo.launch.py world:=$HOME/ros2_ws/src/aws-robomaker-small-house-world/worlds/small_house.world
#bookstore world launch:
# ros2 launch turtlebot gazebo.launch.py world:=$HOME/ros2_ws/src/aws-robomaker-bookstore-world/worlds/bookstore.world

#git clone -b ros2 https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git 
#git clone -b ros2 https://github.com/aws-robotics/aws-robomaker-small-house-world.git
#git clone -b ros2 https://github.com/aws-robotics/aws-robomaker-bookstore-world.git