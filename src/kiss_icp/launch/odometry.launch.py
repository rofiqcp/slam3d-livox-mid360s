import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('kiss_icp')
    default_config = os.path.join(share, 'config', 'mid360s.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('visualize', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('debug_clouds', default_value='true'),
        DeclareLaunchArgument('deskew', default_value='true'),
        DeclareLaunchArgument('base_frame', default_value=''),
        DeclareLaunchArgument('lidar_odom_frame', default_value='odom_lidar'),
        Node(
            package='kiss_icp',
            executable='kiss_icp_node',
            name='kiss_icp_node',
            output='screen',
            remappings=[('pointcloud_topic', LaunchConfiguration('topic'))],
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool),
                    'publish_odom_tf': ParameterValue(
                        LaunchConfiguration('publish_tf'), value_type=bool),
                    'publish_debug_clouds': ParameterValue(
                        LaunchConfiguration('debug_clouds'), value_type=bool),
                    'data.deskew': ParameterValue(
                        LaunchConfiguration('deskew'), value_type=bool),
                    'base_frame': LaunchConfiguration('base_frame'),
                    'lidar_odom_frame': LaunchConfiguration('lidar_odom_frame'),
                    'invert_odom_tf': False,
                },
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='kiss_icp_rviz',
            output='screen',
            arguments=['-d', os.path.join(share, 'rviz', 'kiss_icp.rviz')],
            condition=IfCondition(LaunchConfiguration('visualize')),
        ),
    ])
