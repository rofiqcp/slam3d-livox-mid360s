import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    share = get_package_share_directory('fast_lio')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'mid360s.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('omp_threads', default_value='4'),
        DeclareLaunchArgument('publish_odom_tf', default_value='true'),
        Node(
            package='fast_lio', executable='fastlio_mapping', name='fastlio_mapping',
            output='screen',
            additional_env={
                'OMP_NUM_THREADS': LaunchConfiguration('omp_threads'),
                'OPENBLAS_NUM_THREADS': '1',
                'MKL_NUM_THREADS': '1',
            },
            parameters=[LaunchConfiguration('config'), {
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'publish_odom_tf': ParameterValue(LaunchConfiguration('publish_odom_tf'),value_type=bool)
            }],
            remappings=[
                ('/Odometry', '/lio/odometry'),
                ('/path', '/lio/path'),
                ('/cloud_registered', '/lio/cloud_registered'),
                ('/cloud_registered_body', '/lio/cloud_body'),
                ('/Laser_map', '/lio/map'),
            ],
        )
    ])
