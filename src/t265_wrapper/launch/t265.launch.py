import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    install = LaunchConfiguration('t265_install')
    default_params = os.path.join(
        get_package_share_directory('t265_wrapper'), 'config', 't265.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            't265_install',
            default_value=os.path.expanduser('~/t265_project/install'),
            description='Install prefix of librealsense v2.50 (built with Python bindings)'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(
            package='t265_wrapper',
            executable='t265_node',
            name='t265',
            namespace='t265',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            # Only this process sees librealsense v2.50; the apt ros-jazzy-librealsense2
            # (no T265 support) stays the default everywhere else.
            additional_env={
                'PYTHONPATH': [install, '/lib/python3.12/pyrealsense2:',
                               EnvironmentVariable('PYTHONPATH', default_value='')],
                'LD_LIBRARY_PATH': [install, '/lib:',
                                    EnvironmentVariable('LD_LIBRARY_PATH', default_value='')],
            },
        ),
    ])
