from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([Node(
        package="laser_projection_ros",
        executable="laser_projection_node",
        parameters=[{"backend": "udp", "path_topic": "/plan"}],
    )])
