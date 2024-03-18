import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('rosbag_time_converter'),
        'config',
        'rosbag_time_converter.yaml'
        )

    node = Node(
        package='rosbag_time_converter',
        name='rosbag_time_converter',
        executable='rosbag_time_converter',
        parameters=[config]
    )
    ld.add_action(node)
    return ld
