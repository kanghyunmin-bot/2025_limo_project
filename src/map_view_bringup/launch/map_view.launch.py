#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    namespace      = LaunchConfiguration('namespace')
    use_namespace  = LaunchConfiguration('use_namespace')
    use_sim_time   = LaunchConfiguration('use_sim_time')
    map_yaml       = LaunchConfiguration('map')
    rviz           = LaunchConfiguration('rviz')
    rviz_config    = LaunchConfiguration('rviz_config')

    declare_namespace = DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace')
    declare_use_namespace = DeclareLaunchArgument('use_namespace', default_value='false', description='Apply namespace')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use /clock')
    declare_map = DeclareLaunchArgument('map', description='Full path to a nav2 map .yaml file')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz2 as well')
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(os.path.dirname(__file__), '..', 'rviz', 'map_only.rviz'),
        description='Path to RViz2 config'
    )

    stdout_linebuf = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    grouped = GroupAction([
        PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml,
                'use_sim_time': use_sim_time,
                'frame_id': 'map',
                'topic_name': 'map'
            }]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'bond_timeout': 0.0,
                'node_names': ['map_server']
            }]
        ),

        Node(
            condition=IfCondition(rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),
    ])

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf)
    ld.add_action(declare_namespace)
    ld.add_action(declare_use_namespace)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map)
    ld.add_action(declare_rviz)
    ld.add_action(declare_rviz_config)
    ld.add_action(grouped)
    return ld
