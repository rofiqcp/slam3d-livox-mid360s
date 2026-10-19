import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _check_map(context, *args, **kwargs):
    path=LaunchConfiguration('map_pcd').perform(context)
    if not path or not os.path.isfile(path):
        raise RuntimeError('P5 requires explicit saved Stage4 map PCD: map_pcd:=/path/to/stage4_map_*.pcd')
    return []

def generate_launch_description():
    manager=get_package_share_directory('livox_mid360_manager')
    reloc=get_package_share_directory('slam3d_relocalizer')
    return LaunchDescription([
        DeclareLaunchArgument('map_pcd',default_value=''),
        DeclareLaunchArgument('poses_csv',default_value=''),
        DeclareLaunchArgument('start_sensor',default_value='true'),
        DeclareLaunchArgument('start_lio',default_value='true'),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('publish_odom_tf',default_value='true'),
        DeclareLaunchArgument('auto_global_on_start',default_value='true'),
        DeclareLaunchArgument('publish_tf',default_value='true'),
        OpaqueFunction(function=_check_map),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(manager,'launch','stage3.launch.py')),
            launch_arguments={
                'start_sensor':LaunchConfiguration('start_sensor'),
                'use_sim_time':LaunchConfiguration('use_sim_time'),
                'rviz':'false',
                'publish_odom_tf':LaunchConfiguration('publish_odom_tf'),
            }.items(),condition=IfCondition(LaunchConfiguration('start_lio'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(reloc,'launch','relocalize.launch.py')),
            launch_arguments={
                'map_pcd':LaunchConfiguration('map_pcd'),
                'poses_csv':LaunchConfiguration('poses_csv'),
                'auto_global_on_start':LaunchConfiguration('auto_global_on_start'),
                'publish_tf':LaunchConfiguration('publish_tf'),
                'use_sim_time':LaunchConfiguration('use_sim_time'),
            }.items()),
        Node(package='rviz2',executable='rviz2',name='stage5_rviz',
             arguments=['-d',os.path.join(manager,'rviz','stage5.rviz')],
             output='screen',condition=IfCondition(LaunchConfiguration('rviz')))
    ])