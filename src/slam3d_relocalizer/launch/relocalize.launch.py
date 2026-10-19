import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    share=get_package_share_directory('slam3d_relocalizer')
    root=os.environ.get('LIVOX_MID360_ROOT',os.path.expanduser('~'))
    return LaunchDescription([
      DeclareLaunchArgument('map_pcd',default_value=''),
      DeclareLaunchArgument('poses_csv',default_value=''),
      DeclareLaunchArgument('config',default_value=os.path.join(share,'config','stage5.yaml')),
      DeclareLaunchArgument('use_sim_time',default_value='false'),
      DeclareLaunchArgument('auto_global_on_start',default_value='true'),
      DeclareLaunchArgument('publish_tf',default_value='true'),
      DeclareLaunchArgument('pose_output_dir',
        default_value=os.path.join(os.environ.get('SLAM3D_DATA',os.path.expanduser('~/.local/share/slam3d-mid360s')), 'results','stage5','poses')),
      Node(package='slam3d_relocalizer',executable='relocalizer_node',
        name='relocalizer_node',output='screen',
        parameters=[LaunchConfiguration('config'),{
          'map_pcd':LaunchConfiguration('map_pcd'),
          'poses_csv':LaunchConfiguration('poses_csv'),
          'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool),
          'auto_global_on_start':ParameterValue(LaunchConfiguration('auto_global_on_start'),value_type=bool),
          'use_tf':ParameterValue(LaunchConfiguration('publish_tf'),value_type=bool),
          'pose_output_dir':LaunchConfiguration('pose_output_dir')
        }])
    ])
