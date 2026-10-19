from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def _setup(context, *args, **kwargs):
    bag = LaunchConfiguration('bag').perform(context)
    rate = LaunchConfiguration('rate').perform(context)
    clock = LaunchConfiguration('clock').perform(context).lower() in ('1', 'true', 'yes', 'on')

    if not bag:
        raise RuntimeError('bag:=/path/to/bag is required')

    cmd = ['ros2', 'bag', 'play', bag, '--rate', rate]
    if clock:
        cmd += ['--clock']

    return [ExecuteProcess(cmd=cmd, output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value=''),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('clock', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
