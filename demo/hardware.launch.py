"""Factory drivers + filtered IMU + EKF, with exactly one instance each."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    here=Path(__file__).resolve().parent;vendor=Path(get_package_share_directory('turn_on_wheeltec_robot'))/'launch'
    config=yaml.safe_load((here.parent/'config/vehicle.yaml').read_text())
    lidar=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(vendor/'wheeltec_lidar.launch.py')))
    scan=[lidar]
    if config['antenna_filter']['enabled']:
        scan=[GroupAction([SetRemap(src='/scan',dst='/scan_raw'),lidar]),ExecuteProcess(cmd=['python3',str(here/'self_return_filter.py')],output='screen')]
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(vendor/'base_serial.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(vendor/'imu_processor.launch.py'))),
        Node(package='joint_state_publisher',executable='joint_state_publisher',name='joint_state_publisher'),
        Node(package='robot_localization',executable='ekf_node',name='nav_ekf_filter_node',parameters=[str(here/'assets/demo_ekf_nav.yaml')],remappings=[('/odometry/filtered','odom_combined')],output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(vendor/'robot_mode_description.launch.py'))),*scan])
