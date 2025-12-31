"""Launch file for Sawppy control nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'max_linear_velocity',
            default_value='0.5',
            description='Maximum linear velocity (m/s)'
        ),
        DeclareLaunchArgument(
            'max_angular_velocity',
            default_value='1.85',
            description='Maximum angular velocity (rad/s)'
        ),

        # Ackermann controller
        Node(
            package='sawppy_control',
            executable='ackermann_controller',
            name='ackermann_controller',
            output='screen',
            parameters=[{
                'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
                'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
                'wheel_radius': 0.06,
                'publish_rate': 20.0,
            }]
        ),

        # Odometry node
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
    ])
