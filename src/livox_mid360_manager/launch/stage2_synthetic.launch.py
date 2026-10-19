import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    manager_share = get_package_share_directory('livox_mid360_manager')
    output_root = os.environ.get('LIVOX_MID360_ROOT', os.path.expanduser('~'))
    default_result_dir = os.path.join(
        output_root, 'results', 'stage2', 'runs',
        'synthetic_' + datetime.now().strftime('%Y%m%d_%H%M%S'))


    synthetic_node = Node(
        package='livox_mid360_manager',
        executable='stage2_synthetic_lidar',
        name='stage2_synthetic_lidar',
        output='screen',
    )

    stage2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(manager_share, 'launch', 'stage2.launch.py')),
        launch_arguments={
            'start_sensor': 'false',
            'input_topic': '/stage2/synthetic/lidar',
            'ground_truth_topic': '/stage2/synthetic/ground_truth',
            'publish_tf': 'false',
            'deskew': 'true',
            'debug_clouds': LaunchConfiguration('debug_clouds'),
            'rviz': LaunchConfiguration('rviz'),
            'use_sim_time': 'false',
            'result_dir': LaunchConfiguration('result_dir'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('debug_clouds', default_value='false'),
        DeclareLaunchArgument('result_dir', default_value=default_result_dir),
        synthetic_node,
        stage2,
    ])
