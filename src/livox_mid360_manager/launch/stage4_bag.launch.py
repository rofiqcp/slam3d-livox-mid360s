import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    manager=get_package_share_directory('livox_mid360_manager')
    root=os.environ.get('LIVOX_MID360_ROOT',os.path.expanduser('~'))
    return LaunchDescription([
        DeclareLaunchArgument('bag',default_value=''),
        DeclareLaunchArgument('rate',default_value='1.0'),
        DeclareLaunchArgument('rviz',default_value='false'),
        DeclareLaunchArgument('startup_delay',default_value='5.0'),
        DeclareLaunchArgument('ground_truth_topic',default_value=''),
        DeclareLaunchArgument('enable_loop_closure',default_value='true'),
        DeclareLaunchArgument('lio_config',default_value=os.path.join(root,'config','stage3_fast_lio.yaml')),
        DeclareLaunchArgument('p4_config',default_value=os.path.join(root,'src','slam3d_pose_graph','config','stage4.yaml')),
        DeclareLaunchArgument('map_output_dir',default_value=os.path.join(os.environ.get('SLAM3D_DATA',os.path.expanduser('~/.local/share/slam3d-mid360s')), 'results','stage4','maps')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(manager,'launch','stage3_bag.launch.py')),
            launch_arguments={
                'bag':LaunchConfiguration('bag'),'rate':LaunchConfiguration('rate'),
                'startup_delay':LaunchConfiguration('startup_delay'),'rviz':'false',
                'ground_truth_topic':LaunchConfiguration('ground_truth_topic')
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(manager,'launch','stage4.launch.py')),
            launch_arguments={
                'start_sensor':'false','start_lio':'false',
                'use_sim_time':'true','rviz':LaunchConfiguration('rviz'),
                'enable_loop_closure':LaunchConfiguration('enable_loop_closure'),
                'lio_config':LaunchConfiguration('lio_config'),
                'p4_config':LaunchConfiguration('p4_config'),
                'map_output_dir':LaunchConfiguration('map_output_dir'),
                'ground_truth_topic':LaunchConfiguration('ground_truth_topic')
            }.items())
    ])