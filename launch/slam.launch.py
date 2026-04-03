import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    bringup_dir = get_package_share_directory('turtlebot')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    slam_params = os.path.join(bringup_dir, 'config', 'mapper_params_online_async.yaml')
    slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')

    configured_slam_params = RewrittenYaml(
        source_file=slam_params,
        root_key='',
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True,
    )

    slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={'slam_params_file': configured_slam_params}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'),
        slam_toolbox_cmd,
    ])
