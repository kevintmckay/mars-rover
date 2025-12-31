"""
Main launch file for Sawppy rover.

Launches all core nodes for autonomous operation:
- Robot state publisher (URDF)
- Servo bus driver
- Ackermann controller
- Odometry
- Sensors (optional)
- AI-Link (optional)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    description_dir = get_package_share_directory('sawppy_description')
    hardware_dir = get_package_share_directory('sawppy_hardware')
    control_dir = get_package_share_directory('sawppy_control')
    ai_dir = get_package_share_directory('sawppy_ai')

    # URDF file
    urdf_file = os.path.join(description_dir, 'urdf', 'sawppy.urdf.xacro')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sensors = LaunchConfiguration('use_sensors')
    use_ai = LaunchConfiguration('use_ai')
    serial_port = LaunchConfiguration('serial_port')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'use_sensors',
            default_value='true',
            description='Launch sensor nodes'
        ),
        DeclareLaunchArgument(
            'use_ai',
            default_value='true',
            description='Launch AI-Link node'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for LX-16A servos'
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file]),
                'use_sim_time': use_sim_time,
            }]
        ),

        # Servo bus driver
        Node(
            package='sawppy_hardware',
            executable='servo_bus_node',
            name='servo_bus_node',
            output='screen',
            parameters=[{
                'port': serial_port,
                'baudrate': 115200,
                'publish_rate': 20.0,
                'read_positions': True,
            }]
        ),

        # Ackermann controller
        Node(
            package='sawppy_control',
            executable='ackermann_controller',
            name='ackermann_controller',
            output='screen',
            parameters=[{
                'max_linear_velocity': 0.5,
                'max_angular_velocity': 1.85,
                'wheel_radius': 0.06,
                'publish_rate': 20.0,
            }]
        ),

        # Odometry
        Node(
            package='sawppy_control',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            parameters=[{
                'wheel_radius': 0.06,
                'publish_rate': 20.0,
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
            }]
        ),

        # AI-Link (conditional)
        Node(
            package='sawppy_ai',
            executable='ai_link_node',
            name='ai_link_node',
            output='screen',
            condition=IfCondition(use_ai),
            parameters=[{
                'host': '192.168.1.100',
                'port': 11434,
                'use_tls': True,
                'model': 'qwen2.5:32b-instruct-q4_K_M',
                'timeout': 120.0,
            }]
        ),
    ])
