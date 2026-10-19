import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _setup(context, *args, **kwargs):
    root = os.environ.get('LIVOX_MID360_ROOT', '')
    bag_root = LaunchConfiguration('bag_root').perform(context)
    name = LaunchConfiguration('name').perform(context)
    input_topic = LaunchConfiguration('input_topic').perform(context)
    ground_truth_topic = LaunchConfiguration('ground_truth_topic').perform(context)
    include_debug = _bool(LaunchConfiguration('include_debug_clouds').perform(context))
    max_size_mb = int(LaunchConfiguration('max_bag_size_mb').perform(context))

    if not bag_root:
        bag_root = os.path.join(root, 'bags', 'stage2') if root else os.path.expanduser('~/livox_bags/stage2')
    os.makedirs(bag_root, exist_ok=True)

    output = os.path.join(bag_root, f"{name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}")

    cmd = [
        'ros2', 'bag', 'record',
        '--storage', 'mcap',
        '--compression-mode', 'file',
        '--compression-format', 'zstd',
        '--output', output,
    ]
    if max_size_mb > 0:
        cmd += ['--max-bag-size', str(max_size_mb * 1024 * 1024)]

    topics = [
        input_topic,
        '/livox/imu',
        '/kiss/odometry',
        '/kiss/processing_time_ms',
        '/tf',
        '/tf_static',
        '/diagnostics',
    ]
    if include_debug:
        topics += ['/kiss/frame', '/kiss/keypoints', '/kiss/local_map']
    if ground_truth_topic:
        topics.append(ground_truth_topic)

    cmd += topics

    return [
        LogInfo(msg=f'Stage2 benchmark recording -> {output}'),
        ExecuteProcess(cmd=cmd, output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag_root', default_value=''),
        DeclareLaunchArgument('name', default_value='kiss_baseline'),
        DeclareLaunchArgument('input_topic', default_value='/livox/lidar',
                              description='Raw LiDAR topic, configurable for synthetic tests.'),
        DeclareLaunchArgument('ground_truth_topic', default_value='',
                              description='Optional ground-truth odometry topic for benchmarking.'),
        DeclareLaunchArgument('include_debug_clouds', default_value='false'),
        DeclareLaunchArgument('max_bag_size_mb', default_value='4096'),
        OpaqueFunction(function=_setup),
    ])
