import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    manager_share = get_package_share_directory('livox_mid360_manager')
    kiss_share = get_package_share_directory('kiss_icp')
    manager_launch = os.path.join(manager_share, 'launch')
    kiss_launch = os.path.join(kiss_share, 'launch')
    stage2_cfg = os.path.join(manager_share, 'config', 'stage2.yaml')
    output_root = os.environ.get('LIVOX_MID360_ROOT', os.path.expanduser('~'))
    default_result_dir = os.path.join(
        output_root, 'results', 'stage2', 'runs',
        datetime.now().strftime('%Y%m%d_%H%M%S'))


    sensor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(manager_launch, 'sensor.launch.py')),
        launch_arguments={
            'rviz': 'false',
            'monitor': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_sensor')),
    )

    kiss = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kiss_launch, 'odometry.launch.py')),
        launch_arguments={
            'topic': LaunchConfiguration('input_topic'),
            'visualize': LaunchConfiguration('rviz'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'debug_clouds': LaunchConfiguration('debug_clouds'),
            'deskew': LaunchConfiguration('deskew'),
            'base_frame': '',
            'lidar_odom_frame': 'odom_lidar',
        }.items(),
    )

    evaluator = Node(
        package='livox_mid360_manager',
        executable='stage2_odom_evaluator',
        name='stage2_odom_evaluator',
        output='screen',
        parameters=[
            stage2_cfg,
            {
                'ground_truth_topic': LaunchConfiguration('ground_truth_topic'),
                'csv_path': PathJoinSubstitution([
                    LaunchConfiguration('result_dir'), 'kiss_icp_odometry.csv']),
                'summary_path': PathJoinSubstitution([
                    LaunchConfiguration('result_dir'), 'kiss_icp_odometry.summary.yaml']),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_sensor', default_value='true',
            description='Start the live Livox driver. False for bag/synthetic input.'),
        DeclareLaunchArgument('input_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('result_dir', default_value=default_result_dir,
                              description='Unique per-run directory; avoids overwriting benchmark results.'),
        DeclareLaunchArgument('ground_truth_topic', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('debug_clouds', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('deskew', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        sensor,
        kiss,
        evaluator,
    ])
