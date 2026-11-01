"""Vendor detectors/filter; this kit owns goals, never vendor assigner + adapter together."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    here=Path(__file__).resolve().parent;vendor=Path(get_package_share_directory('wheeltec_robot_rrt'))
    params=[str(vendor/'config/exploration_params.yaml'),{'use_sim_time':False}]
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(here/'keepout.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(vendor/'launch/navigation/nav2_bringup_launch.py')),launch_arguments={'slam':'True','map':str(vendor/'map_data/image_map.yaml'),'params_file':str(here/'navigation.yaml'),'use_sim_time':'false','autostart':'True','use_composition':'False'}.items()),
        *[Node(package='wheeltec_robot_rrt',executable=name,name=name,parameters=params,output='screen') for name in ('local_rrt','global_rrt','filter')],
        ExecuteProcess(cmd=['python3','-u',str(here/'frontier_tasks.py'),'--ros-args','-p','start_paused:=true'],output='screen')])
