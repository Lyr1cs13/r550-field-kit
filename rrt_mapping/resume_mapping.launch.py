"""Factory sensor/EKF/SLAM composition with native serialized-map startup."""
import hashlib
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    here = Path(__file__).resolve().parent
    value = LaunchConfiguration('checkpoint').perform(context)
    if not value:
        value = json.loads((here.parent / 'maps/latest_checkpoint.json').read_text())['directory']
    checkpoint = Path(value).expanduser().resolve()
    manifest = json.loads((checkpoint / 'manifest.json').read_text())
    for name, expected in manifest['sha256'].items():
        if hashlib.sha256((checkpoint / name).read_bytes()).hexdigest() != expected:
            raise RuntimeError(f'Checkpoint checksum mismatch: {name}')
    actions = []
    if LaunchConfiguration('with_hardware').perform(context).lower() == 'true':
        vendor = Path(get_package_share_directory('turn_on_wheeltec_robot')) / 'launch'
        actions.extend([
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(vendor / 'turn_on_wheeltec_robot.launch.py')),
                                     launch_arguments={'robot_nav': 'True'}.items()),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(vendor / 'wheeltec_lidar.launch.py'))),
        ])
    actions.append(Node(package='slam_toolbox', executable='sync_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[str(checkpoint / 'native_slam_params.yaml'), {
            'map_file_name': str(checkpoint / manifest['graph_prefix']),
            'map_start_pose': [float(v) for v in manifest['startup_pose']],
            'map_start_at_dock': False,
            # M10P scans arrive faster than the factory 10 Hz EKF TF.
            # Keep enough native filter slots to wait for matching TF.
            'scan_queue_size': 10,
        }], remappings=[('odom', 'odom_combined')]))
    # No motion is requested. The first native scan establishes map->odom;
    # mapping then pauses until a human verifies alignment and starts a region.
    actions.append(TimerAction(period=3.0, actions=[ExecuteProcess(
        cmd=['python3', str(here / 'session_checkpoint.py'), 'pause'], output='screen')]))
    if LaunchConfiguration('with_navigation').perform(context).lower() == 'true':
        actions.append(TimerAction(period=6.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(here / 'rrt_mapping.launch.py')))]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('checkpoint', default_value=''),
        DeclareLaunchArgument('with_hardware', default_value='true'),
        DeclareLaunchArgument('with_navigation', default_value='true'),
        OpaqueFunction(function=setup),
    ])
