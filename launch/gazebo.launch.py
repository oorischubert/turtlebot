import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node



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
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items() 
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


    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
    )

    bookstore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory(
                'aws_robomaker_bookstore_world'), '/launch/bookstore.launch.py']
        )
    )


    # Code for delaying a node (I haven't tested how effective it is)
    # 
    # First add the below lines to imports
    # from launch.actions import RegisterEventHandler
    # from launch.event_handlers import OnProcessExit
    #
    # Then add the following below the current diff_drive_spawner
    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[diff_drive_spawner],
    #     )
    # )
    #
    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner



    # Launch them all!
    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
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