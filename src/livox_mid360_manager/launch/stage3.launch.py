import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    manager = get_package_share_directory('livox_mid360_manager')
    lio = get_package_share_directory('fast_lio')
    root = os.environ.get('LIVOX_MID360_ROOT', os.path.expanduser('~'))
    workspace_config = os.path.join(root, 'config', 'stage3_fast_lio.yaml')
    default_lio_config = (
        workspace_config if os.path.isfile(workspace_config)
        else os.path.join(lio, 'config', 'mid360s.yaml'))
    out = os.path.join(os.environ.get('SLAM3D_DATA',os.path.expanduser('~/.local/share/slam3d-mid360s')), 'results', 'stage3', 'runs',
                       datetime.now().strftime('%Y%m%d_%H%M%S'))

    return LaunchDescription([
        DeclareLaunchArgument('start_sensor', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('publish_odom_tf', default_value='true'),
        DeclareLaunchArgument('lio_config',
            default_value=default_lio_config),
        DeclareLaunchArgument('ground_truth_topic', default_value=''),
        DeclareLaunchArgument('result_dir', default_value=out),
        LogInfo(msg='P3 FAST-LIO: LiDAR PointCloud2 timestamp + Livox IMU. '
                    'Initial extrinsics are uncalibrated; live accuracy is provisional.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(manager, 'launch', 'sensor.launch.py')),
            launch_arguments={'rviz': 'false', 'monitor': 'false'}.items(),
            condition=IfCondition(LaunchConfiguration('start_sensor'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lio, 'launch', 'mapping.launch.py')),
            launch_arguments={
                'config': LaunchConfiguration('lio_config'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_odom_tf': LaunchConfiguration('publish_odom_tf')
            }.items()),
        Node(
            package='livox_mid360_manager', executable='livox_diagnostics',
            name='livox_mid360_diagnostics', output='screen',
            parameters=[os.path.join(manager, 'config', 'stage1.yaml')],
        ),
        Node(
            package='livox_mid360_manager', executable='stage2_odom_evaluator',
            name='stage3_odom_evaluator', output='screen',
            parameters=[{
                'backend': 'fast_lio',
                'odom_topic': '/lio/odometry',
                'path_topic': '/lio/evaluation_path',
                'ground_truth_topic': LaunchConfiguration('ground_truth_topic'),
                'csv_path': PathJoinSubstitution([
                    LaunchConfiguration('result_dir'), 'fast_lio_odometry.csv']),
                'summary_path': PathJoinSubstitution([
                    LaunchConfiguration('result_dir'), 'fast_lio_odometry.summary.yaml']),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            }]
        ),
        Node(
            package='rviz2', executable='rviz2',
            name='stage3_rviz', output='screen',
            arguments=['-d', os.path.join(lio, 'rviz', 'fastlio.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
