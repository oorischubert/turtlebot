import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )

    bringup_dir = get_package_share_directory('turtlebot')  # Replace 'turtlebot' with your package name
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # Path to your custom mapper_params_online_async.yaml file
    slam_params = os.path.join(bringup_dir, 'config', 'mapper_params_online_async.yaml')
    slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
    map_file_path = os.path.join(bringup_dir, 'maps', 'bedroom_serial')

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'map_file_name': map_file_path
    }

    configured_slam_params = RewrittenYaml(
        source_file=slam_params,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # Include the online_async_launch.py launch file, passing the slam_params_file argument
    slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={'slam_params_file': configured_slam_params}.items()
    )

    # Create the launch description and add the slam_toolbox_cmd action
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(slam_toolbox_cmd)

    return ld