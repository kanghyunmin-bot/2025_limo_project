#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('path_follower_pkg')
    default_params_file = os.path.join(pkg_share, 'config', 'follower_params.yaml')
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to parameter file'
    )
    
    follower_node = Node(
        package='path_follower_pkg',
        executable='follower_node',
        name='interactive_path_follower',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        emulate_tty=True,
    )

    return LaunchDescription([
        params_file_arg,
        follower_node,
    ])
