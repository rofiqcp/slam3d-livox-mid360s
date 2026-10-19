import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _setup(context, *args, **kwargs):
    root = os.environ.get('LIVOX_MID360_ROOT', os.path.expanduser('~'))
    bag_root = LaunchConfiguration('bag_root').perform(context)
    bag_root = bag_root or os.path.join(root, 'bags', 'stage3')
    name = LaunchConfiguration('name').perform(context)
    gt_topic = LaunchConfiguration('ground_truth_topic').perform(context)
    os.makedirs(bag_root, exist_ok=True)
    output = os.path.join(bag_root, name + '_' + datetime.now().strftime('%Y%m%d_%H%M%S'))

    topics = [
        '/livox/lidar', '/livox/imu', '/lio/odometry', '/lio/path',
        '/lio/cloud_registered', '/diagnostics', '/tf', '/tf_static',
    ]
    if gt_topic:
        topics.append(gt_topic)

    cmd = [
        'ros2', 'bag', 'record', '-s', 'mcap', '-o', output,
        '--compression-mode', 'file', '--compression-format', 'zstd',
        '--max-bag-size', str(int(LaunchConfiguration('max_size_mb').perform(context)) * 1024 * 1024)
    ] + topics

    return [
        LogInfo(msg='Stage3 raw IMU + LIO benchmark recording: ' + output),
        ExecuteProcess(cmd=cmd, output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='fast_lio_mid360s'),
        DeclareLaunchArgument('bag_root', default_value=''),
        DeclareLaunchArgument('ground_truth_topic', default_value=''),
        DeclareLaunchArgument('max_size_mb', default_value='4096'),
        OpaqueFunction(function=_setup),
    ])
