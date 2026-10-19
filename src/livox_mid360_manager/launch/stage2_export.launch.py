import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_output = os.path.join(
        os.environ.get('LIVOX_MID360_ROOT', os.path.expanduser('~')),
        'datasets', 'stage2', 'kitti_scans')

    return LaunchDescription([
        DeclareLaunchArgument('topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('output_dir', default_value=default_output),
        DeclareLaunchArgument('max_scans', default_value='0'),
        Node(
            package='livox_mid360_manager',
            executable='cloud_to_kitti',
            name='stage2_cloud_to_kitti',
            output='screen',
            parameters=[{
                'topic': LaunchConfiguration('topic'),
                'output_dir': LaunchConfiguration('output_dir'),
                'max_scans': ParameterValue(
                    LaunchConfiguration('max_scans'), value_type=int),
            }],
        ),
    ])
