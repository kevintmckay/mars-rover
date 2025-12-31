"""Launch file for servo bus node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port for LX-16A bus'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='Serial baud rate'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='20.0',
            description='Joint state publish rate in Hz'
        ),

        Node(
            package='sawppy_hardware',
            executable='servo_bus_node',
            name='servo_bus_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'read_positions': True,
            }]
        ),
    ])
