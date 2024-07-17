import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{'channel_type':'serial',
                         'serial_port': '/dev/ttyUSB_rplidar',
                         'serial_baudrate': '115200',
                         'frame_id': 'laser_frame',
                         'inverted': False,
                         'angle_compensate': True,
                         'scan_mode' : 'Boost'
                         }],
            output='screen'),
            
    ])
