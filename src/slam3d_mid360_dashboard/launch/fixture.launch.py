"""Software-only P4 synthetic fixture plus C++ map browser; no driver, no motors."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    graph=get_package_share_directory("slam3d_pose_graph")
    dash=get_package_share_directory("slam3d_mid360_dashboard")
    return LaunchDescription([
        DeclareLaunchArgument("web_port",default_value="5001"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(graph,"launch","loop_fixture.launch.py"))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(dash,"launch","mapping.launch.py")),
            launch_arguments={"start_pipeline":"false","web_port":LaunchConfiguration("web_port")}.items())
    ])
