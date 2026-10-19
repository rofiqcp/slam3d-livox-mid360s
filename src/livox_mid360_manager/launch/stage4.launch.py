import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    manager = get_package_share_directory('livox_mid360_manager')
    graph = get_package_share_directory('slam3d_pose_graph')
    root = os.environ.get('LIVOX_MID360_ROOT', os.path.expanduser('~'))
    return LaunchDescription([
        DeclareLaunchArgument('start_sensor', default_value='true'),
        DeclareLaunchArgument('start_lio', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_loop_closure', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('ground_truth_topic', default_value=''),
        DeclareLaunchArgument('p4_result_dir', default_value=os.path.join(
            root, 'results', 'stage4', 'runs',
            datetime.now().strftime('%Y%m%d_%H%M%S'))),
        DeclareLaunchArgument('lio_config', default_value=os.path.join(root,'config','stage3_fast_lio.yaml')),
        DeclareLaunchArgument('p4_config', default_value=os.path.join(graph,'config','stage4.yaml')),
        DeclareLaunchArgument('map_output_dir', default_value=os.path.join(os.environ.get('SLAM3D_DATA',os.path.expanduser('~/.local/share/slam3d-mid360s')), 'results','stage4','maps')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(manager,'launch','stage3.launch.py')),
            launch_arguments={
                'start_sensor': LaunchConfiguration('start_sensor'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'rviz': 'false',
                'lio_config': LaunchConfiguration('lio_config')
            }.items(),condition=IfCondition(LaunchConfiguration('start_lio'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(graph,'launch','pose_graph.launch.py')),
            launch_arguments={
                'config': LaunchConfiguration('p4_config'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'enable_loop_closure': LaunchConfiguration('enable_loop_closure'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                'map_output_dir': LaunchConfiguration('map_output_dir')
            }.items()),
        Node(package='livox_mid360_manager',
             executable='stage2_odom_evaluator',
             name='stage4_odom_evaluator', output='screen',
             parameters=[{
                 'backend': 'stage4_pose_graph',
                 'odom_topic': '/p4/optimized_odometry',
                 'path_topic': '/p4/evaluation_path',
                 'ground_truth_topic': LaunchConfiguration('ground_truth_topic'),
                 'csv_path': PathJoinSubstitution([
                     LaunchConfiguration('p4_result_dir'),'stage4_corrected.csv']),
                 'summary_path': PathJoinSubstitution([
                     LaunchConfiguration('p4_result_dir'),'stage4_corrected.summary.yaml']),
                 'use_sim_time': ParameterValue(
                     LaunchConfiguration('use_sim_time'),value_type=bool),
             }]),
        Node(package='rviz2', executable='rviz2', name='stage4_rviz',
             arguments=['-d',os.path.join(manager,'rviz','stage4.rviz')],
             output='screen', condition=IfCondition(LaunchConfiguration('rviz')))
    ])