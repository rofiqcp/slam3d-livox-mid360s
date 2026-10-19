"""Dashboard standalone, attaches to existing ROS2 P4 backend if running."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    root=get_package_share_directory("slam3d_mid360_dashboard")
    return LaunchDescription([
        DeclareLaunchArgument("web_port",default_value="5001"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(root,"launch","mapping.launch.py")),
            launch_arguments={"start_pipeline":"false",
                              "web_port":LaunchConfiguration("web_port")}.items())
    ])
