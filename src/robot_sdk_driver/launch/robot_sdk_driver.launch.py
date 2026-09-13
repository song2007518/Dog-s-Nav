import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_sdk_driver')
    config_file = os.path.join(pkg_share, 'config', 'robot_sdk_driver.yaml')

    return LaunchDescription([
        Node(
            package='robot_sdk_driver',
            executable='robot_sdk_driver',
            name='robot_sdk_driver',
            output='screen',
            parameters=[config_file],
        )
    ])
