"""Launch file for displaying Sawppy URDF in RViz."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('sawppy_description')

    urdf_file = os.path.join(pkg_dir, 'urdf', 'sawppy.urdf.xacro')
    rviz_config = os.path.join(pkg_dir, 'config', 'display.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('use_gui')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Use joint_state_publisher_gui'
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

        # Joint state publisher (GUI for testing)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=LaunchConfiguration('use_gui'),
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
