import json
import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('livox_mid360_manager')
    config_path = LaunchConfiguration('config').perform(context)
    rviz_enabled = _as_bool(LaunchConfiguration('rviz').perform(context))
    monitor_enabled = _as_bool(LaunchConfiguration('monitor').perform(context))

    with open(config_path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f) or {}

    device = cfg.get('device', {})
    network = cfg.get('network', {})
    driver = cfg.get('driver', {})
    ports = cfg.get('ports', {})

    lidar_ip = str(device.get('lidar_ip', '')).strip()
    host_ip = str(network.get('host_ip', '')).strip()
    interface = str(network.get('interface', '')).strip()

    if not lidar_ip or not host_ip:
        raise RuntimeError(
            'livox_runtime.yaml must contain device.lidar_ip and network.host_ip. '
            'Run livox_discover first or edit the fallback YAML.')

    driver_json = {
        'lidar_summary_info': {
            'lidar_type': 8
        },
        'Mid360s': {
            'lidar_net_info': {
                'cmd_data_port': int(ports.get('cmd_data_port', 56100)),
                'push_msg_port': int(ports.get('push_msg_port', 56200)),
                'point_data_port': int(ports.get('point_data_port', 56300)),
                'imu_data_port': int(ports.get('imu_data_port', 56400)),
                'log_data_port': int(ports.get('log_data_port', 56500)),
            },
            'host_net_info': [
                {
                    'host_ip': host_ip,
                    'cmd_data_port': int(ports.get('host_cmd_data_port', 56101)),
                    'push_msg_port': int(ports.get('host_push_msg_port', 56201)),
                    'point_data_port': int(ports.get('host_point_data_port', 56301)),
                    'imu_data_port': int(ports.get('host_imu_data_port', 56401)),
                    'log_data_port': int(ports.get('host_log_data_port', 56501)),
                }
            ]
        },
        'lidar_configs': [
            {
                'ip': lidar_ip,
                'pcl_data_type': 1,
                'pattern_mode': 0,
                'extrinsic_parameter': {
                    'roll': 0.0,
                    'pitch': 0.0,
                    'yaw': 0.0,
                    'x': 0,
                    'y': 0,
                    'z': 0
                }
            }
        ]
    }

    runtime_dir = os.path.join(tempfile.gettempdir(), 'livox_mid360_manager')
    os.makedirs(runtime_dir, exist_ok=True)
    driver_config_path = os.path.join(runtime_dir, 'MID360s_runtime.json')
    with open(driver_config_path, 'w', encoding='utf-8') as f:
        json.dump(driver_json, f, indent=2)

    frame_id = str(driver.get('frame_id', 'livox_frame'))
    publish_freq = float(driver.get('publish_freq', 10.0))
    xfer_format = int(driver.get('xfer_format', 0))
    point_topic = str(driver.get('point_topic', '/livox/lidar'))
    imu_topic = str(driver.get('imu_topic', '/livox/imu'))

    actions = [
        LogInfo(msg=(
            f'Livox MID-360S: interface={interface} host={host_ip} '
            f'lidar={lidar_ip} config={config_path}'
        )),
        LogInfo(msg=f'Generated driver JSON: {driver_config_path}')
    ]

    actions.append(
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            remappings=[('/livox/lidar', point_topic), ('/livox/imu', imu_topic)],
            output='screen',
            parameters=[{
                'xfer_format': xfer_format,
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': publish_freq,
                'output_data_type': 0,
                'frame_id': frame_id,
                'lvx_file_path': '',
                'user_config_path': driver_config_path,
                'cmdline_input_bd_code': 'livox0000000001',
            }]
        )
    )

    if monitor_enabled and xfer_format == 0:
        actions.append(
            Node(
                package='livox_mid360_manager',
                executable='livox_monitor',
                name='livox_mid360_monitor',
                output='screen',
                parameters=[{
                    'point_topic': point_topic,
                    'imu_topic': imu_topic,
                    'report_period': 2.0,
                }]
            )
        )
    elif monitor_enabled:
        actions.append(LogInfo(msg='Monitor disabled because xfer_format is not PointCloud2 (0).'))

    if rviz_enabled:
        rviz_config = os.path.join(pkg_share, 'rviz', 'mid360.rviz')
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
            )
        )

    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('livox_mid360_manager')
    explicit_config = os.environ.get('LIVOX_MID360_CONFIG', '')
    xdg = os.environ.get('XDG_CONFIG_HOME', os.path.expanduser('~/.config'))
    user_config = os.path.join(xdg, 'livox_mid360_manager', 'livox_runtime.yaml')
    fallback_config = os.path.join(pkg_share, 'config', 'livox_runtime.yaml')
    if explicit_config and os.path.exists(explicit_config):
        default_config = explicit_config
    elif os.path.exists(user_config):
        default_config = user_config
    else:
        default_config = fallback_config

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Runtime YAML written by livox_discover or edited manually.'),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz2.'),
        DeclareLaunchArgument(
            'monitor',
            default_value='true',
            description='Start C++ topic-rate/health monitor.'),
        OpaqueFunction(function=_launch_setup),
    ])
