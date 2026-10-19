import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    manager=get_package_share_directory('livox_mid360_manager')
    return LaunchDescription([
        DeclareLaunchArgument('bag',default_value=''),
        DeclareLaunchArgument('map_pcd',default_value=''),
        DeclareLaunchArgument('poses_csv',default_value=''),
        DeclareLaunchArgument('rate',default_value='1.0'),
        DeclareLaunchArgument('startup_delay',default_value='5.0'),
        DeclareLaunchArgument('rviz',default_value='false'),
        DeclareLaunchArgument('auto_global_on_start',default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(manager,'launch','stage3_bag.launch.py')),
            launch_arguments={
                'bag':LaunchConfiguration('bag'),
                'rate':LaunchConfiguration('rate'),
                'startup_delay':LaunchConfiguration('startup_delay'),
                'rviz':'false'
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(manager,'launch','stage5.launch.py')),
            launch_arguments={
                'map_pcd':LaunchConfiguration('map_pcd'),
                'poses_csv':LaunchConfiguration('poses_csv'),
                'start_sensor':'false','start_lio':'false',
                'use_sim_time':'true',
                'auto_global_on_start':LaunchConfiguration('auto_global_on_start'),
                'rviz':LaunchConfiguration('rviz')
            }.items())
    ])