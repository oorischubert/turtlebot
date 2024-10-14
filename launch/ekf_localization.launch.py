from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
import os
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

#   -<><><>- WORK IN PROGRESS -<><><>-

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    bringup_dir = get_package_share_directory('turtlebot')
    ekf_config = os.path.join(
        get_package_share_directory(bringup_dir),
        'config',
        'ekf.yaml'
    )
    amcl_params = os.path.join(
        get_package_share_directory(bringup_dir),
        'config',
        'amcl_params.yaml'
    )
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('/scan', '/scan')] # Adjust if scan topic is different

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params, {'use_sim_time': use_sim_time}],
        remappings=remappings 
    )

    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[ekf_config, {'use_sim_time': use_sim_time}])

    ld = LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        amcl_node,
        robot_localization_node
    ])

    return ld