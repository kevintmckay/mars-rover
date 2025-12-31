"""Launch file for Sawppy AI-Link node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'host',
            default_value='192.168.1.100',
            description='Ollama server IP address'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='11434',
            description='Ollama server port'
        ),
        DeclareLaunchArgument(
            'use_tls',
            default_value='true',
            description='Use TLS encryption'
        ),
        DeclareLaunchArgument(
            'model',
            default_value='qwen2.5:32b-instruct-q4_K_M',
            description='Default Ollama model'
        ),

        Node(
            package='sawppy_ai',
            executable='ai_link_node',
            name='ai_link_node',
            output='screen',
            parameters=[{
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'use_tls': LaunchConfiguration('use_tls'),
                'model': LaunchConfiguration('model'),
                'timeout': 120.0,
                'verify_ssl': True,
            }]
        ),
    ])
