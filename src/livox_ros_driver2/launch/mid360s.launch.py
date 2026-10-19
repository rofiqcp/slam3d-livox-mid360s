import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('livox_ros_driver2')
    default_config = os.path.join(share, 'config', 'MID360s_config.json')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('frame_id', default_value='livox_frame'),
        DeclareLaunchArgument('publish_freq', default_value='10.0'),
        DeclareLaunchArgument('xfer_format', default_value='0', description='0=Livox PointCloud2 XYZRTLT, 1=CustomMsg, 2=standard PointCloud2 XYZI'),
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=[{
                'xfer_format': ParameterValue(LaunchConfiguration('xfer_format'), value_type=int),
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': ParameterValue(LaunchConfiguration('publish_freq'), value_type=float),
                'output_data_type': 0,
                'frame_id': LaunchConfiguration('frame_id'),
                'lvx_file_path': '',
                'user_config_path': LaunchConfiguration('config'),
                'cmdline_input_bd_code': 'livox0000000001'
            }]
        )
    ])
