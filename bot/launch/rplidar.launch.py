import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='rplidar_ros', # rplidar_composition
            executable='rplidar_node',
            name='rplidar_node', # rplidar_composition
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                # ls /dev/serial/by-path
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard', # Sensitivity
            }]
        )
    ])
