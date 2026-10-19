import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    manager = get_package_share_directory('livox_mid360_manager')
    root = os.environ.get('LIVOX_MID360_ROOT', os.path.expanduser('~'))
    out = os.path.join(os.environ.get('SLAM3D_DATA',os.path.expanduser('~/.local/share/slam3d-mid360s')), 'results', 'stage3', 'runs',
                       'synthetic_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('result_dir', default_value=out),
        Node(package='livox_mid360_manager',
             executable='stage2_synthetic_lidar', name='stage3_synthetic_lidar',
             output='screen',
             parameters=[{
                 'topic': '/livox/lidar',
                 'ground_truth_topic': '/stage3/synthetic/ground_truth',
             }]),
        Node(package='livox_mid360_manager',
             executable='stage3_synthetic_imu', name='stage3_synthetic_imu',
             output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(manager, 'launch', 'stage3.launch.py')),
            launch_arguments={
                'start_sensor': 'false', 'rviz': LaunchConfiguration('rviz'),
                'ground_truth_topic': '/stage3/synthetic/ground_truth',
                'result_dir': LaunchConfiguration('result_dir'),
            }.items()),
    ])
