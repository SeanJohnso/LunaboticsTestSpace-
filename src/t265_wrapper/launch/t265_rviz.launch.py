"""RViz view of the T265 odometry. Runs on the dev machine (the Pi is headless).

Only starts RViz: run t265.launch.py on the Pi for the data.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_rviz = os.path.join(
        get_package_share_directory('t265_wrapper'), 'rviz', 't265.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ),
    ])
