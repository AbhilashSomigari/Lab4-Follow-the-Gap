"""Launch the Follow the Gap node with parameters from config/params.yaml."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('gap_follow'), 'config', 'params.yaml')
    return LaunchDescription([
        Node(
            package='gap_follow',
            executable='gap_follow_node',
            name='gap_follow_node',
            output='screen',
            parameters=[params],
        ),
    ])
