"""Native Nav2 keepout servers. No publication to /map or /tf."""
import json
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    checkpoint = Path(__file__).resolve().parent.parent/'state/mapping'
    return LaunchDescription([
        DeclareLaunchArgument('mask_yaml', default_value=str(checkpoint / 'threshold_keepout.yaml')),
        Node(package='nav2_map_server', executable='map_server', name='threshold_mask_server',
             parameters=[{'use_sim_time': False, 'yaml_filename': LaunchConfiguration('mask_yaml'),
                          'topic_name': '/threshold_keepout_mask', 'frame_id': 'map'}], output='screen'),
        Node(package='nav2_map_server', executable='costmap_filter_info_server', name='threshold_filter_info_server',
             parameters=[{'use_sim_time': False, 'type': 0, 'filter_info_topic': '/threshold_filter_info',
                          'mask_topic': '/threshold_keepout_mask', 'base': 0.0, 'multiplier': 1.0}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_threshold',
             parameters=[{'use_sim_time': False, 'autostart': True,
                          'node_names': ['threshold_mask_server', 'threshold_filter_info_server']}], output='screen'),
    ])
