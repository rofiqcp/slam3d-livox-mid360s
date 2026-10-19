import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _bag_player(context, *args, **kwargs):
    bag = LaunchConfiguration('bag').perform(context)
    if not bag:
        raise RuntimeError('bag:=/path/to/rosbag is required')

    delay = float(LaunchConfiguration('startup_delay').perform(context))
    return [
        TimerAction(period=delay, actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'play', bag,
                    '--rate', LaunchConfiguration('rate'),
                    '--clock', LaunchConfiguration('clock_hz'),
                ],
                output='screen',
            )
        ])
    ]


def generate_launch_description():
    share = get_package_share_directory('livox_mid360_manager')
    output_root = os.environ.get('LIVOX_MID360_ROOT', os.path.expanduser('~'))
    default_result_dir = os.path.join(
        output_root, 'results', 'stage2', 'runs',
        'bag_' + datetime.now().strftime('%Y%m%d_%H%M%S'))


    stage2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'stage2.launch.py')),
        launch_arguments={
            'start_sensor': 'false',
            'input_topic': LaunchConfiguration('input_topic'),
            'rviz': LaunchConfiguration('rviz'),
            'debug_clouds': LaunchConfiguration('debug_clouds'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'deskew': LaunchConfiguration('deskew'),
            'use_sim_time': 'true',
            'ground_truth_topic': LaunchConfiguration('ground_truth_topic'),
            'result_dir': LaunchConfiguration('result_dir'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value=''),
        DeclareLaunchArgument('input_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('ground_truth_topic', default_value=''),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('result_dir', default_value=default_result_dir),
        DeclareLaunchArgument('startup_delay', default_value='4.0',
                              description='Wait for KISS-ICP subscriptions before starting rosbag.'),
        DeclareLaunchArgument('clock_hz', default_value='100.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('debug_clouds', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('deskew', default_value='true'),
        stage2,
        OpaqueFunction(function=_bag_player),
    ])
