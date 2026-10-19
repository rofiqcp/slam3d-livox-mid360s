import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _assert_navigation_safe(context, *args, **kwargs):
    sim = LaunchConfiguration('simulation_only_override').perform(context).lower() == 'true'
    if sim and (
        LaunchConfiguration('start_sensor').perform(context).lower() == 'true' or
        LaunchConfiguration('start_lio').perform(context).lower() == 'true'):
        raise RuntimeError('simulation_only_override is forbidden with the live sensor/LIO pipeline. Use the standalone P6 fixture.')
    if LaunchConfiguration('start_nav2').perform(context).lower() != 'true':
        return []
    if sim:
        raise RuntimeError('Use stage6_nav2_fixture.launch.py to test Nav2 with a synthetic sensor; never bypass calibration in live stack.')
    path=LaunchConfiguration('extrinsics').perform(context)
    with open(path,'r',encoding='utf-8') as f:
        calib=yaml.safe_load(f)['extrinsics']
    if str(calib.get('status','')).lower() not in ('verified','calibrated'):
        raise RuntimeError('P6 Nav2 launch is blocked until base_link->livox_frame is physically calibrated. Use fixture for testing; never label uncalibrated identity as real.')
    return []


def generate_launch_description():
    bridge=get_package_share_directory('slam3d_nav2_bridge')
    manager=get_package_share_directory('livox_mid360_manager')
    nav2=get_package_share_directory('nav2_bringup')
    root=os.environ.get('LIVOX_MID360_ROOT',os.path.expanduser('~'))
    return LaunchDescription([
        DeclareLaunchArgument('map_pcd',default_value=''),
        DeclareLaunchArgument('poses_csv',default_value=''),
        DeclareLaunchArgument('extrinsics',default_value=os.path.join(root,'config','extrinsics.yaml')),
        DeclareLaunchArgument('start_sensor',default_value='true'),
        DeclareLaunchArgument('start_lio',default_value='true'),
        DeclareLaunchArgument('start_nav2',default_value='false'),
        DeclareLaunchArgument('rviz',default_value='false'),
        DeclareLaunchArgument('simulation_only_override',default_value='false'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('nav2_params',default_value=os.path.join(bridge,'config','nav2_mid360s.yaml')),
        OpaqueFunction(function=_assert_navigation_safe),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(manager,'launch','stage5.launch.py')),
            launch_arguments={
                'map_pcd':LaunchConfiguration('map_pcd'),
                'poses_csv':LaunchConfiguration('poses_csv'),
                'start_sensor':LaunchConfiguration('start_sensor'),
                'start_lio':LaunchConfiguration('start_lio'),
                'publish_odom_tf':'false',
                'use_sim_time':LaunchConfiguration('use_sim_time'),
                'rviz':LaunchConfiguration('rviz')
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bridge,'launch','bridge_only.launch.py')),
            launch_arguments={
                'map_pcd':LaunchConfiguration('map_pcd'),
                'extrinsics':LaunchConfiguration('extrinsics'),
                'use_sim_time':LaunchConfiguration('use_sim_time'),
                'simulation_only_override':LaunchConfiguration('simulation_only_override')
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2,'launch','navigation_launch.py')),
            condition=IfCondition(LaunchConfiguration('start_nav2')),
            launch_arguments={
                'use_sim_time':LaunchConfiguration('use_sim_time'),
                'params_file':LaunchConfiguration('nav2_params'),
                'autostart':'true',
                'use_composition':'False',
            }.items())
    ])
