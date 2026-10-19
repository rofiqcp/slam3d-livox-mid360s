import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    share=get_package_share_directory('slam3d_pose_graph')
    root=os.environ.get('LIVOX_MID360_ROOT', os.path.expanduser('~'))
    return LaunchDescription([
        Node(package='slam3d_pose_graph', executable='loop_fixture',
             name='stage4_loop_fixture', output='screen'),
        Node(package='livox_mid360_manager',executable='stage2_odom_evaluator',
             name='p4_fixture_evaluator',output='screen',
             parameters=[{
                 'backend':'p4_fixture',
                 'odom_topic':'/p4/optimized_odometry',
                 'path_topic':'/p4/fixture_evaluation_path',
                 'ground_truth_topic':'/p4/fixture/ground_truth',
                 'csv_path':os.path.join(os.environ.get('SLAM3D_DATA',os.path.expanduser('~/.local/share/slam3d-mid360s')), 'results','stage4','fixture_optimized.csv'),
                 'summary_path':os.path.join(os.environ.get('SLAM3D_DATA',os.path.expanduser('~/.local/share/slam3d-mid360s')), 'results','stage4','fixture_optimized.summary.yaml'),
             }]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share,'launch','pose_graph.launch.py')),
            launch_arguments={
                'config':os.path.join(share,'config','loop_fixture.yaml'),
                'map_output_dir':os.path.join(os.environ.get('SLAM3D_DATA',os.path.expanduser('~/.local/share/slam3d-mid360s')), 'results','stage4','fixture_maps')
            }.items())
    ])
