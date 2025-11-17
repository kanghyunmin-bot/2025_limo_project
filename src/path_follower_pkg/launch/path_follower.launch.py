from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('path_follower_pkg'), 'config')
    params_file = os.path.join(config_dir, 'path_follower_params.yaml')
    
    # Path Follower Node
    follower_node = Node(
        package='path_follower_pkg',
        executable='follower_node',
        name='interactive_path_follower',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/odom', '/odom'),
            ('/clicked_point', '/clicked_point'),
            ('/initialpose', '/initialpose'),
        ],
    )
    
    # Control Panel GUI
    control_panel = Node(
        package='path_follower_pkg',
        executable='control_panel',
        name='path_follower_gui',
        output='screen',
    )
    
    return LaunchDescription([
        follower_node,
        control_panel,
    ])
