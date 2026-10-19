import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bridge=get_package_share_directory('slam3d_nav2_bridge')
    nav2=get_package_share_directory('nav2_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('frames',default_value='180'),
        DeclareLaunchArgument('fixture_rate',default_value='5.0'),
        DeclareLaunchArgument('blackout_start',default_value='0'),
        DeclareLaunchArgument('blackout_frames',default_value='0'),
        DeclareLaunchArgument('start_nav2',default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bridge,'launch','stage6_fixture.launch.py')),
            launch_arguments={
                'frames':LaunchConfiguration('frames'),
                'fixture_rate':LaunchConfiguration('fixture_rate'),
                'blackout_start':LaunchConfiguration('blackout_start'),
                'blackout_frames':LaunchConfiguration('blackout_frames'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2,'launch','navigation_launch.py')),
            condition=IfCondition(LaunchConfiguration('start_nav2')),
            launch_arguments={
                'use_sim_time':'false',
                'params_file':os.path.join(bridge,'config','nav2_mid360s_fixture.yaml'),
                'autostart':'true',
                'use_composition':'False',
                'use_respawn':'False'
            }.items())
    ])
