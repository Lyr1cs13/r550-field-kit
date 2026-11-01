"""Fresh or checkpoint SLAM; manual mode has no autonomous goal producer."""
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    here=Path(__file__).resolve().parent;root=here.parent
    automatic=LaunchConfiguration('automatic').perform(context).lower()=='true'
    checkpoint=LaunchConfiguration('checkpoint').perform(context)
    hardware=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root/'demo/hardware.launch.py')))
    if checkpoint:
        slam=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(here/'resume_mapping.launch.py')),launch_arguments={'checkpoint':checkpoint,'with_hardware':'false','with_navigation':'false'}.items())
    else:
        params=Path(get_package_share_directory('wheeltec_slam_toolbox'))/'config/mapper_params_online_sync.yaml'
        slam=Node(package='slam_toolbox',executable='sync_slam_toolbox_node',name='slam_toolbox',parameters=[str(params),{'scan_queue_size':10}],remappings=[('odom','odom_combined')],output='screen')
    actions=[hardware,TimerAction(period=3.,actions=[slam])]
    if automatic:actions.append(TimerAction(period=8.,actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(str(here/'rrt_mapping.launch.py')))]))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('automatic',default_value='false'),DeclareLaunchArgument('checkpoint',default_value=''),OpaqueFunction(function=setup)])
