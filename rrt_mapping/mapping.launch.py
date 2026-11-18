"""Fresh or checkpoint SLAM; manual mode has no autonomous goal producer."""
import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    here=Path(__file__).resolve().parent;root=here.parent
    automatic=LaunchConfiguration('automatic').perform(context).lower()=='true'
    panel=LaunchConfiguration('panel').perform(context).lower()=='true'
    checkpoint=LaunchConfiguration('checkpoint').perform(context)
    with_hardware=LaunchConfiguration('with_hardware').perform(context).lower()=='true'
    hardware=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root/'demo/hardware.launch.py')))
    if checkpoint:
        slam=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(here/'resume_mapping.launch.py')),
            launch_arguments={'checkpoint':checkpoint,'with_hardware':'false','with_navigation':'false',
                              'panel':str(panel).lower()}.items())
    else:
        params=Path(get_package_share_directory('wheeltec_slam_toolbox'))/'config/mapper_params_online_sync.yaml'
        settings={'scan_queue_size':10}
        if panel:
            settings.update(map_update_interval=1.,minimum_travel_distance=.12,minimum_travel_heading=.18)
        remaps=[('odom','odom_combined')]+([('/tf','/site_slam_tf')] if panel else [])
        slam=Node(package='slam_toolbox',executable='sync_slam_toolbox_node',name='slam_toolbox',
                  parameters=[str(params),settings],remappings=remaps,output='screen')
    actions=([hardware] if with_hardware else [])+[TimerAction(period=3.,actions=[slam])]
    if panel:
        state=Path(os.environ['R550_MAPPING_STATE'])
        nav=yaml.safe_load((here/'navigation.yaml').read_text())
        controller=nav['controller_server']['ros__parameters']
        controller['goal_checker'].update(xy_goal_tolerance=.12,yaw_goal_tolerance=.18)
        # A predictable low-speed native controller is also used for unfinished-map returns.
        controller['FollowPath']={
            'plugin':'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
            'desired_linear_vel':.10,'lookahead_dist':.45,'use_velocity_scaled_lookahead_dist':False,
            'rotate_to_heading_angular_vel':.22,'max_angular_accel':.4,'allow_reversing':False,
            'use_rotate_to_heading':True,'min_approach_linear_velocity':.035,
            'approach_velocity_scaling_dist':.28,'use_collision_detection':True,
            'transform_tolerance':.25,'max_allowed_time_to_collision_up_to_carrot':1.,
            'inflation_cost_scaling_factor':3.,
        }
        config=state/'panel_navigation.yaml'
        config.write_text(yaml.safe_dump(nav),encoding='utf-8')
        actions += [
            ExecuteProcess(cmd=['python3',str(root/'demo/motion_gate.py'),'--ros-args',
                                '-p','site_mapping:=true'],output='screen'),
            TimerAction(period=7.,actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(here/'rrt_mapping.launch.py')),
                launch_arguments={'panel':'true','params_file':str(config),
                                  'mask_yaml':str(state/'threshold_keepout.yaml')}.items())]),
        ]
    elif automatic:
        actions.append(TimerAction(period=8.,actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(str(here/'rrt_mapping.launch.py')))]))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('automatic',default_value='false'),
                              DeclareLaunchArgument('panel',default_value='false'),
                              DeclareLaunchArgument('with_hardware',default_value='true'),
                              DeclareLaunchArgument('checkpoint',default_value=''),OpaqueFunction(function=setup)])
