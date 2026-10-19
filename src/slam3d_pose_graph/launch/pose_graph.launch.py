import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    share = get_package_share_directory('slam3d_pose_graph')
    root = os.environ.get('LIVOX_MID360_ROOT', os.path.expanduser('~'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'stage4.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('omp_threads', default_value='4'),
        DeclareLaunchArgument('enable_loop_closure', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('map_output_dir',
            default_value=os.path.join(os.environ.get('SLAM3D_DATA',os.path.expanduser('~/.local/share/slam3d-mid360s')), 'results', 'stage4', 'maps')),
        Node(package='slam3d_pose_graph', executable='pose_graph_node',
            name='pose_graph_node', output='screen',
            additional_env={
                'OMP_NUM_THREADS': LaunchConfiguration('omp_threads'),
                'OPENBLAS_NUM_THREADS': '1',
                'MKL_NUM_THREADS': '1',
            },
            parameters=[LaunchConfiguration('config'), {
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'enable_loop_closure': ParameterValue(LaunchConfiguration('enable_loop_closure'), value_type=bool),
                'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
                'map_output_dir': LaunchConfiguration('map_output_dir')
            }])
    ])