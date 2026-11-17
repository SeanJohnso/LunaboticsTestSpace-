from glob import glob

from setuptools import setup

package_name = 't265_wrapper'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='ROS 2 wrapper for the Intel RealSense T265',
    license='TODO',
    entry_points={
        'console_scripts': [
            't265_node = t265_wrapper.t265_node:main',
        ],
    },
)
