import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('livox_mid360_manager')
    launch_dir = os.path.join(pkg, 'launch')
    stage1_cfg = os.path.join(pkg, 'config', 'stage1.yaml')

    sensor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sensor.launch.py')),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'monitor': 'false',
        }.items(),
    )

    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'static_tf.launch.py')),
        condition=IfCondition(LaunchConfiguration('publish_tf')),
    )

    diagnostics = Node(
        package='livox_mid360_manager',
        executable='livox_diagnostics',
        name='livox_mid360_diagnostics',
        output='screen',
        parameters=[stage1_cfg],
    )

    stationary = Node(
        package='livox_mid360_manager',
        executable='imu_stationary_stats',
        name='livox_imu_stationary_stats',
        output='screen',
        parameters=[stage1_cfg],
        condition=IfCondition(LaunchConfiguration('stationary_stats')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='false',
            description='Publish base_link -> livox_frame from extrinsics.yaml. Keep false until calibrated.'),
        DeclareLaunchArgument(
            'stationary_stats',
            default_value='false',
            description='Collect a 10-second stationary IMU baseline and save YAML.'),
        sensor,
        static_tf,
        diagnostics,
        stationary,
    ])
