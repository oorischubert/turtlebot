import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.event_handlers import OnProcessExit


def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='turtlebot' #<--- CHANGE ME

    world_path = PathJoinSubstitution([
    FindPackageShare('aws_robomaker_small_warehouse_world'),
    'worlds',
    'no_roof_small_warehouse',
    'no_roof_small_warehouse.world'
    ])

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diffbot_base_controller/cmd_vel_unstamped')]
        )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items() 
             )
    
    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')
    
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[controller_params_file],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot',
                                    '-x', '0',
                                    '-y', '0',
                                    '-z', '0.04'],
                        output='screen',
                        )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner", #FOXY: add .py to spawner!
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner", #FOXY: add .py to spawner!
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
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

    bookstore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory(
                'aws_robomaker_bookstore_world'), '/launch/bookstore.launch.py']
        )
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        gazebo,
        spawn_entity,
        delayed_controller_manager,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delayed_joint_broad_spawner,
        #bookstore
    ])


# . /usr/share/gazebo/setup.bash
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