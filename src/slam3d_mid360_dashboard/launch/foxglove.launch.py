"""Read-only Foxglove WebSocket bridge for Livox MID-360S diagnostics and mapping."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="mid360_foxglove_bridge",
            output="screen",
            parameters=[{
                "address": "127.0.0.1",
                "port": 8765,
                "num_threads": 4,
                "use_sim_time": False,
                "topic_whitelist": [
                    "^/livox/(lidar|imu)$",
                    "^/lio/.*$",
                    "^/p4/.*$",
                    "^/mid360/.*$",
                    "^/diagnostics$",
                    "^/tf(_static)?$",
                    "^/parameter_events$",
                    "^/foxglove_bridge/sysinfo$"
                ],
                "service_whitelist": ["(?!)"],
                "client_topic_whitelist": ["(?!)"],
                "param_whitelist": ["(?!)"],
                "capabilities": ["connectionGraph", "assets"],
                "send_buffer_limit": 20000000,
                "min_qos_depth": 1,
                "max_qos_depth": 10,
            }],
        ),
    ])
