import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    bringup_dir = get_package_share_directory('turtlebot')
    default_rviz_config_path = os.path.join(bringup_dir, 'config/main.rviz')
    view_bot_rviz_config_path = os.path.join(bringup_dir, 'config/view_bot.rviz')
    mapping_rviz_config_path = os.path.join(bringup_dir, 'config/mapping.rviz')

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', mapping_rviz_config_path],
        )
    
    return LaunchDescription([
        rviz
    ])