"""C++ monitoring + P4 corrected map preview; opt-in existing FAST-LIO/P4 backend."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    share=get_package_share_directory("slam3d_mid360_dashboard")
    manager=get_package_share_directory("livox_mid360_manager")
    root=os.environ.get("LIVOX_MID360_ROOT",os.path.expanduser("~"))
    params=os.path.join(share,"config","dashboard.yaml")
    stage4=os.path.join(manager,"launch","stage4.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("start_pipeline",default_value="true"),
        DeclareLaunchArgument("start_sensor",default_value="false"),
        DeclareLaunchArgument("use_sim_time",default_value="false"),
        DeclareLaunchArgument("enable_loop_closure",default_value="true"),
        DeclareLaunchArgument("lio_config",default_value=os.path.join(root,"config","stage3_fast_lio.yaml")),
        DeclareLaunchArgument("p4_config",default_value=os.path.join(root,"src","slam3d_pose_graph","config","stage4.yaml")),
        DeclareLaunchArgument("map_output_dir",default_value=os.path.join(os.environ.get('SLAM3D_DATA',os.path.expanduser('~/.local/share/slam3d-mid360s')), 'results',"stage4","maps")),
        DeclareLaunchArgument("web_port",default_value="5001"),
        LogInfo(msg="MID360S dashboard: stage4 corrected map -> preview -> localhost:5001. "
                    "Sensor is not started by default to avoid a duplicate driver."),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(stage4),
            launch_arguments={
                "start_sensor":LaunchConfiguration("start_sensor"),
                "rviz":"false",
                "use_sim_time":LaunchConfiguration("use_sim_time"),
                 "enable_loop_closure":LaunchConfiguration("enable_loop_closure"),
                "lio_config":LaunchConfiguration("lio_config"),
                "p4_config":LaunchConfiguration("p4_config"),
                "map_output_dir":LaunchConfiguration("map_output_dir")
            }.items(),condition=IfCondition(LaunchConfiguration("start_pipeline"))),
        Node(package="slam3d_mid360_dashboard",executable="cloud_filter",
            name="cloud_filter",parameters=[params,{"use_sim_time":ParameterValue(
                LaunchConfiguration("use_sim_time"),value_type=bool)}],output="screen"),
        Node(package="slam3d_mid360_dashboard",executable="map_preview",
            name="map_preview",parameters=[params,{"use_sim_time":ParameterValue(
                LaunchConfiguration("use_sim_time"),value_type=bool)}],output="screen"),
        Node(package="slam3d_mid360_dashboard",executable="web_dashboard",
            name="web_dashboard",parameters=[params,{"port":ParameterValue(
                LaunchConfiguration("web_port"),value_type=int),
                "use_sim_time":ParameterValue(
                    LaunchConfiguration("use_sim_time"),value_type=bool)}],output="screen"),
    ])
