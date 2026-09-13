import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('custom_nav')
    config_file = os.path.join(pkg_share, 'config', 'custom_nav.yaml')

    return LaunchDescription([
        Node(
            package='custom_nav',
            executable='custom_nav_node',
            name='custom_nav_node',
            output='screen',
            parameters=[config_file],
        )
    ])
