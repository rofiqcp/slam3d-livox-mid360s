import os
from glob import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _setup(context,*args,**kwargs):
    root=os.environ.get('LIVOX_MID360_ROOT','')
    if not root:
        raise RuntimeError('Source workspace env.sh before P6 fixture')
    files=glob(os.path.join(os.environ.get('SLAM3D_DATA',os.path.expanduser('~/.local/share/slam3d-mid360s')), 'results','stage4','fixture_maps','stage4_map_*.pcd'))
    files.sort(key=os.path.getmtime,reverse=True)
    selected=None
    for pcd in files:
        csv=pcd.replace('stage4_map_','stage4_poses_').replace('.pcd','.csv')
        if os.path.isfile(csv):
            selected=pcd;break
    if not selected:
        raise RuntimeError('P6 fixture needs saved matching Stage4 fixture PCD/CSV')
    manager=get_package_share_directory('livox_mid360_manager')
    bridge=get_package_share_directory('slam3d_nav2_bridge')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(manager,'launch','stage5_fixture.launch.py')),
            launch_arguments={
                'rviz':'false',
                'frames':LaunchConfiguration('frames'),
                'fixture_rate':LaunchConfiguration('fixture_rate'),
                'blackout_start':LaunchConfiguration('blackout_start'),
                'blackout_frames':LaunchConfiguration('blackout_frames')
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bridge,'launch','bridge_only.launch.py')),
            launch_arguments={
                'map_pcd':selected,
                'simulation_only_override':'true',
                'use_sim_time':'false'
            }.items())
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('frames',default_value='130'),
        DeclareLaunchArgument('fixture_rate',default_value='5.0'),
        DeclareLaunchArgument('blackout_start',default_value='0'),
        DeclareLaunchArgument('blackout_frames',default_value='0'),
        OpaqueFunction(function=_setup)
    ])
