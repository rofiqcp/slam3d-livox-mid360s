import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    root = os.environ.get('LIVOX_MID360_ROOT', os.path.expanduser('~'))
    default_output = os.path.join(root, 'datasets', 'stage2', 'ct_ply')

    return LaunchDescription([
        DeclareLaunchArgument('topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('output_root', default_value=default_output),
        DeclareLaunchArgument('sequence_id', default_value='0'),
        DeclareLaunchArgument('max_scans', default_value='0'),
        Node(
            package='livox_mid360_manager',
            executable='cloud_to_ct_ply',
            name='stage2_cloud_to_ct_ply',
            output='screen',
            parameters=[{
                'topic': LaunchConfiguration('topic'),
                'output_root': LaunchConfiguration('output_root'),
                'sequence_id': ParameterValue(
                    LaunchConfiguration('sequence_id'), value_type=int),
                'max_scans': ParameterValue(
                    LaunchConfiguration('max_scans'), value_type=int),
            }],
        ),
    ])
