import os
from glob import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def _setup(context,*args,**kwargs):
    root=os.environ.get('LIVOX_MID360_ROOT','')
    if not root:
        raise RuntimeError('Source workspace env.sh first')
    base=os.path.join(os.environ.get('SLAM3D_DATA',os.path.expanduser('~/.local/share/slam3d-mid360s')), 'results','stage4','fixture_maps')
    files=sorted(glob(os.path.join(base,'stage4_map_*.pcd')))
    if not files:
        raise RuntimeError('No saved P4 fixture PCD. Run ros2 launch slam3d_pose_graph loop_fixture.launch.py and /p4/save_map first.')
    # Choose the most recently modified valid PCD/CSV pair.
    files.sort(key=os.path.getmtime,reverse=True)
    selected=None
    for pcd in files:
        csv=pcd.replace('stage4_map_','stage4_poses_').replace('.pcd','.csv')
        if os.path.isfile(csv):
            selected=(pcd,csv);break
    if not selected:
        raise RuntimeError('No matching P4 fixture pose CSV')
    pcd,csv=selected
    reloc=get_package_share_directory('slam3d_relocalizer')
    manager=get_package_share_directory('livox_mid360_manager')
    return [
        Node(package='slam3d_relocalizer',executable='map_fixture',
             name='stage5_fixture',output='screen',
             parameters=[{
                 'map_pcd':pcd,'poses_csv':csv,
                 'rate_hz':ParameterValue(LaunchConfiguration('fixture_rate'),value_type=float),
                 'frames':ParameterValue(LaunchConfiguration('frames'),value_type=int),
                 'start_index':ParameterValue(LaunchConfiguration('start_index'),value_type=int),
                 'inject_start':ParameterValue(LaunchConfiguration('inject_start'),value_type=int),
                 'inject_frames':ParameterValue(LaunchConfiguration('inject_frames'),value_type=int),
                 'blackout_start':ParameterValue(LaunchConfiguration('blackout_start'),value_type=int),
                 'blackout_frames':ParameterValue(LaunchConfiguration('blackout_frames'),value_type=int)
             }]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(manager,'launch','stage5.launch.py')),
            launch_arguments={
                'map_pcd':pcd,'poses_csv':csv,
                'start_sensor':'false','start_lio':'false',
                'auto_global_on_start':'true','rviz':LaunchConfiguration('rviz')
            }.items()),
        Node(package='livox_mid360_manager',executable='stage2_odom_evaluator',
             name='stage5_fixture_evaluator',output='screen',
             parameters=[{
                 'backend':'p5_saved_map',
                 'odom_topic':'/p5/localized_odometry',
                 'path_topic':'/p5/fixture_path',
                 'ground_truth_topic':'/p5/fixture/ground_truth',
                 'csv_path':os.path.join(os.environ.get('SLAM3D_DATA',os.path.expanduser('~/.local/share/slam3d-mid360s')), 'results','stage5','fixture_localized.csv'),
                 'summary_path':os.path.join(os.environ.get('SLAM3D_DATA',os.path.expanduser('~/.local/share/slam3d-mid360s')), 'results','stage5','fixture_localized.summary.yaml')
             }])
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz',default_value='false'),
        DeclareLaunchArgument('fixture_rate',default_value='5.0'),
        DeclareLaunchArgument('frames',default_value='100'),
        DeclareLaunchArgument('start_index',default_value='0'),
        DeclareLaunchArgument('inject_start',default_value='0'),
        DeclareLaunchArgument('inject_frames',default_value='0'),
        DeclareLaunchArgument('blackout_start',default_value='0'),
        DeclareLaunchArgument('blackout_frames',default_value='0'),
        OpaqueFunction(function=_setup)
    ])