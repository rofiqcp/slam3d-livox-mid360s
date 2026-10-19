import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    manager=get_package_share_directory('livox_mid360_manager')
    return LaunchDescription([
        DeclareLaunchArgument('rviz',default_value='false'),
        DeclareLaunchArgument('enable_loop_closure',default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(manager,'launch','stage3_synthetic.launch.py')),
            launch_arguments={'rviz':'false'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(manager,'launch','stage4.launch.py')),
            launch_arguments={
                'start_sensor':'false','start_lio':'false',
                'ground_truth_topic':'/stage3/synthetic/ground_truth',
                'rviz':LaunchConfiguration('rviz'),
                'enable_loop_closure':LaunchConfiguration('enable_loop_closure')
            }.items())
    ])