"""Vendor detectors/filter; this kit owns goals, never vendor assigner + adapter together."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    here=Path(__file__).resolve().parent
    panel=LaunchConfiguration('panel').perform(context).lower()=='true'
    keepout=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(here/'keepout.launch.py')),
                                    launch_arguments={'mask_yaml':LaunchConfiguration('mask_yaml')}.items())
    nav_params=LaunchConfiguration('params_file')
    if panel:
        # The panel submits known paths directly. No BT recovery action can
        # publish a second velocity source during a mode switch.
        return [keepout,
                Node(package='nav2_controller',executable='controller_server',name='controller_server',
                     parameters=[nav_params],remappings=[('cmd_vel','/site_nav_cmd_vel')],output='screen'),
                Node(package='nav2_planner',executable='planner_server',name='planner_server',
                     parameters=[nav_params],output='screen'),
                Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_navigation',
                     parameters=[{'autostart':True,'node_names':['controller_server','planner_server']}],output='screen')]
    vendor=Path(get_package_share_directory('wheeltec_robot_rrt'))
    params=[str(vendor/'config/exploration_params.yaml'),{'use_sim_time':False}]
    return [keepout,
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(vendor/'launch/navigation/nav2_bringup_launch.py')),
                launch_arguments={'slam':'True','map':str(vendor/'map_data/image_map.yaml'),'params_file':nav_params,
                                  'use_sim_time':'false','autostart':'True','use_composition':'False'}.items()),
            *[Node(package='wheeltec_robot_rrt',executable=name,name=name,parameters=params,output='screen')
              for name in ('local_rrt','global_rrt','filter')],
            ExecuteProcess(cmd=['python3','-u',str(here/'frontier_tasks.py'),'--ros-args','-p','start_paused:=true'],
                           output='screen')]


def generate_launch_description():
    here=Path(__file__).resolve().parent
    return LaunchDescription([
        DeclareLaunchArgument('panel',default_value='false'),
        DeclareLaunchArgument('params_file',default_value=str(here/'navigation.yaml')),
        DeclareLaunchArgument('mask_yaml',default_value=str(here.parent/'state/mapping/threshold_keepout.yaml')),
        OpaqueFunction(function=setup)])
