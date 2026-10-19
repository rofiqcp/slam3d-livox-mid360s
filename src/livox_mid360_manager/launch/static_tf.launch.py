import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    path = LaunchConfiguration('extrinsics').perform(context)
    with open(path, 'r', encoding='utf-8') as f:
        cfg = (yaml.safe_load(f) or {}).get('extrinsics', {})

    t = cfg.get('translation_m', {})
    r = cfg.get('rotation_rpy_rad', {})
    parent = str(cfg.get('parent_frame', 'base_link'))
    child = str(cfg.get('child_frame', 'livox_frame'))

    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='livox_static_tf',
            output='screen',
            arguments=[
                '--x', str(t.get('x', 0.0)),
                '--y', str(t.get('y', 0.0)),
                '--z', str(t.get('z', 0.0)),
                '--roll', str(r.get('roll', 0.0)),
                '--pitch', str(r.get('pitch', 0.0)),
                '--yaw', str(r.get('yaw', 0.0)),
                '--frame-id', parent,
                '--child-frame-id', child,
            ],
        )
    ]


def generate_launch_description():
    pkg = get_package_share_directory('livox_mid360_manager')
    explicit = os.environ.get('LIVOX_MID360_EXTRINSICS', '')
    root = os.environ.get('LIVOX_MID360_ROOT', '')
    root_cfg = os.path.join(root, 'config', 'extrinsics.yaml') if root else ''
    fallback = os.path.join(pkg, 'config', 'extrinsics.yaml')

    if explicit and os.path.exists(explicit):
        default = explicit
    elif root_cfg and os.path.exists(root_cfg):
        default = root_cfg
    else:
        default = fallback

    return LaunchDescription([
        DeclareLaunchArgument('extrinsics', default_value=default),
        OpaqueFunction(function=_setup),
    ])
