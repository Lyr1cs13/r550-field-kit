"""Static-map demonstration using the installed factory components. No SLAM/RRT."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    here = Path(__file__).resolve().parent
    base = Path(get_package_share_directory('turn_on_wheeltec_robot'))/'launch'
    nav = Path(get_package_share_directory('wheeltec_nav2'))/'launch'
    config=yaml.safe_load((here/'navigation.yaml').read_text())
    smooth='velocity_smoother' in config
    staged=config.get('demo_arrival',{}).get('ros__parameters',{}).get('enabled',False)
    platform=[IncludeLaunchDescription(PythonLaunchDescriptionSource(str(here/'hardware.launch.py')))]
    navigation=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav/'bringup_launch.py')),
        launch_arguments={'slam':'False','map':str(here/'assets/map.yaml'),
                          'params_file':str(here/'navigation.yaml'),'use_sim_time':'false',
                          'autostart':'True','use_composition':'False'}.items())
    if smooth:
        navigation=GroupAction([SetRemap(src='cmd_vel',dst='/nav_cmd_vel'),navigation])
    extra=[Node(package='nav2_velocity_smoother',executable='velocity_smoother',name='velocity_smoother',
                parameters=[str(here/'navigation.yaml')],
                remappings=[('cmd_vel','/nav_cmd_vel'),('cmd_vel_smoothed','/cmd_vel')],output='screen')] if smooth else []
    return LaunchDescription([
        *platform,
        Node(package='nav2_map_server',executable='map_server',name='threshold_mask_server',
             parameters=[{'use_sim_time':False,'yaml_filename':str(here/'assets/threshold_keepout.yaml'),
                          'topic_name':'/threshold_keepout_mask','frame_id':'map'}],output='screen'),
        Node(package='nav2_map_server',executable='costmap_filter_info_server',name='threshold_filter_info_server',
             parameters=[{'use_sim_time':False,'type':0,'filter_info_topic':'/threshold_filter_info',
                          'mask_topic':'/threshold_keepout_mask','base':0.0,'multiplier':1.0}],output='screen'),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_threshold',
             parameters=[{'use_sim_time':False,'autostart':True,
                          'node_names':['threshold_mask_server','threshold_filter_info_server']+(['velocity_smoother'] if smooth else [])}],output='screen'),
        *extra,
        navigation,
    ])
