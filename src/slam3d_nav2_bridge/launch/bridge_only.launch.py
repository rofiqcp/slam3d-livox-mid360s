import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _setup(context, *args, **kwargs):
    root = os.environ.get('LIVOX_MID360_ROOT', '')
    if not root:
        raise RuntimeError('Source workspace env.sh before starting P6')
    share = get_package_share_directory('slam3d_nav2_bridge')
    cfg = LaunchConfiguration('config').perform(context)
    ex_path = LaunchConfiguration('extrinsics').perform(context)
    pcd = LaunchConfiguration('map_pcd').perform(context)
    sim = LaunchConfiguration('simulation_only_override').perform(context).lower() == 'true'
    if not os.path.isfile(pcd):
        raise RuntimeError('P6 needs an explicit saved P4 map PCD: map_pcd:=/path/to/stage4_map_*.pcd')
    if not os.path.isfile(ex_path):
        raise RuntimeError('Extrinsics YAML missing: ' + ex_path)
    with open(ex_path, 'r', encoding='utf-8') as f:
        extrinsics = yaml.safe_load(f).get('extrinsics', {})
    if extrinsics.get('parent_frame') != 'base_link' or extrinsics.get('child_frame') != 'livox_frame':
        raise RuntimeError('P6 needs the validated base_link -> livox_frame transform')
    xyz = extrinsics.get('translation_m', {})
    rpy = extrinsics.get('rotation_rpy_rad', {})
    xyz = [float(xyz[k]) for k in ('x', 'y', 'z')]
    rpy = [float(rpy[k]) for k in ('roll', 'pitch', 'yaw')]
    calibrated = str(extrinsics.get('status', '')).lower() in ('calibrated', 'verified')
    if sim:
        # Software-only isolated testing. Never attach this topic to physical motors.
        calibration_ok = False
    else:
        calibration_ok = calibrated
    return [
        Node(package='slam3d_nav2_bridge', executable='odom_base_bridge',
             output='screen', name='odom_base_bridge',
             parameters=[cfg, {
                 'sensor_xyz_m': xyz,
                 'sensor_rpy_rad': rpy,
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
             }]),
        Node(package='slam3d_nav2_bridge', executable='localization_stop_gate',
             output='screen', name='localization_stop_gate',
             parameters=[cfg, {
                 'calibration_valid': calibration_ok,
                 'simulation_only_override': sim,
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
             }]),
        Node(package='slam3d_nav2_bridge', executable='pcd_occupancy_map',
             output='screen', name='pcd_occupancy_map',
             parameters=[cfg, {
                 'map_pcd': pcd,
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
             }])
    ]


def generate_launch_description():
    share=get_package_share_directory('slam3d_nav2_bridge')
    root=os.environ.get('LIVOX_MID360_ROOT', os.path.expanduser('~'))
    return LaunchDescription([
        DeclareLaunchArgument('map_pcd',default_value=''),
        DeclareLaunchArgument('config',default_value=os.path.join(share,'config','stage6.yaml')),
        DeclareLaunchArgument('extrinsics',default_value=os.path.join(root,'config','extrinsics.yaml')),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('simulation_only_override',default_value='false'),
        OpaqueFunction(function=_setup),
    ])
