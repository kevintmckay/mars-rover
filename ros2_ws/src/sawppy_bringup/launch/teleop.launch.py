"""
Teleop launch file for Sawppy rover.

Launches robot with keyboard teleop for testing.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('sawppy_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for LX-16A servos'
        ),

        # Include main sawppy launch (without AI to reduce complexity)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'sawppy.launch.py')
            ),
            launch_arguments={
                'use_ai': 'false',
                'use_sensors': 'false',
                'serial_port': LaunchConfiguration('serial_port'),
            }.items()
        ),

        # Teleop twist keyboard
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',
            parameters=[{
                'speed': 0.3,
                'turn': 0.5,
            }]
        ),
    ])
