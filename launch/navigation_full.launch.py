import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_dir = get_package_share_directory('turtlebot')
    launch_dir = os.path.join(pkg_dir, 'launch')

    # EKF for sensor fusion (odom -> base_link TF)
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'ekf_localization.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # SLAM Toolbox for mapping (map -> odom TF)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'slam.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Nav2 navigation stack
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'nav2.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'),
        ekf,
        slam,
        nav2,
    ])

# Usage:
# ros2 launch turtlebot navigation_full.launch.py use_sim_time:=true   (simulation)
# ros2 launch turtlebot navigation_full.launch.py                      (real robot)
