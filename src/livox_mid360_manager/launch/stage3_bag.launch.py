import os
import yaml
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _player(context, *args, **kwargs):
    bag = LaunchConfiguration('bag').perform(context)
    if not os.path.isdir(bag) or not os.path.isfile(os.path.join(bag, 'metadata.yaml')):
        raise RuntimeError('Expected an existing rosbag2 directory with metadata.yaml: ' + bag)
    with open(os.path.join(bag, 'metadata.yaml'), 'r', encoding='utf-8') as f:
        metadata = yaml.safe_load(f) or {}
    info = metadata.get('rosbag2_bagfile_information', {})
    topics_in_bag = {
        item.get('topic_metadata', {}).get('name')
        for item in info.get('topics_with_message_count', [])
        if item.get('message_count', 0) > 0
    }
    required_topics = {'/livox/lidar', '/livox/imu'}
    missing = sorted(required_topics - topics_in_bag)
    if missing:
        raise RuntimeError('FAST-LIO requires real data in both cloud and IMU topics. '
                           'Missing in rosbag2: ' + ', '.join(missing))
    rate = LaunchConfiguration('rate').perform(context)
    delay = float(LaunchConfiguration('startup_delay').perform(context))
    # Replay only raw sensors, never previously recorded /lio/odometry or /tf.
    # Otherwise the benchmark would mix old and freshly estimated trajectories.
    topics = ['/livox/lidar', '/livox/imu']
    gt_topic = LaunchConfiguration('ground_truth_topic').perform(context)
    if gt_topic:
        if gt_topic not in topics:
            if gt_topic not in topics_in_bag:
                raise RuntimeError('Ground-truth topic missing in bag: ' + gt_topic)
            topics.append(gt_topic)
    cmd = ['ros2', 'bag', 'play', bag, '--rate', rate, '--clock', '100',
           '--topics'] + topics
    return [TimerAction(period=delay, actions=[
        ExecuteProcess(cmd=cmd, output='screen')
    ])]


def generate_launch_description():
    manager = get_package_share_directory('livox_mid360_manager')
    root = os.environ.get('LIVOX_MID360_ROOT', os.path.expanduser('~'))
    out = os.path.join(os.environ.get('SLAM3D_DATA',os.path.expanduser('~/.local/share/slam3d-mid360s')), 'results', 'stage3', 'runs',
                       'bag_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value=''),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('startup_delay', default_value='5.0'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('result_dir', default_value=out),
        DeclareLaunchArgument('ground_truth_topic', default_value=''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(manager, 'launch', 'stage3.launch.py')),
            launch_arguments={
                'start_sensor': 'false', 'rviz': LaunchConfiguration('rviz'),
                'use_sim_time': 'true',
                'result_dir': LaunchConfiguration('result_dir'),
                'ground_truth_topic': LaunchConfiguration('ground_truth_topic'),
            }.items()),
        OpaqueFunction(function=_player),
    ])
