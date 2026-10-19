import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration


def _setup(context, *args, **kwargs):
    root = os.environ.get('LIVOX_MID360_ROOT', '')
    bag_root = LaunchConfiguration('bag_root').perform(context)
    name = LaunchConfiguration('name').perform(context)
    storage = LaunchConfiguration('storage').perform(context)
    compression = LaunchConfiguration('compression').perform(context)
    max_size_mb = int(LaunchConfiguration('max_bag_size_mb').perform(context))

    if not bag_root:
        bag_root = os.path.join(root, 'bags') if root else os.path.expanduser('~/livox_bags')
    os.makedirs(bag_root, exist_ok=True)

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output = os.path.join(bag_root, f'{name}_{stamp}')

    cmd = [
        'ros2', 'bag', 'record',
        '--storage', storage,
        '--output', output,
    ]

    if compression.lower() != 'none':
        cmd += ['--compression-mode', 'file', '--compression-format', compression]

    if max_size_mb > 0:
        cmd += ['--max-bag-size', str(max_size_mb * 1024 * 1024)]

    cmd += [
        '/livox/lidar',
        '/livox/imu',
        '/tf',
        '/tf_static',
        '/diagnostics',
    ]

    return [
        LogInfo(msg=f'Recording Stage-1 bag -> {output}'),
        ExecuteProcess(cmd=cmd, output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag_root', default_value=''),
        DeclareLaunchArgument('name', default_value='livox_stage1'),
        DeclareLaunchArgument('storage', default_value='mcap'),
        DeclareLaunchArgument('compression', default_value='zstd'),
        DeclareLaunchArgument(
            'max_bag_size_mb',
            default_value='4096',
            description='Split bag when a file reaches this size; 0 disables splitting.'),
        OpaqueFunction(function=_setup),
    ])
