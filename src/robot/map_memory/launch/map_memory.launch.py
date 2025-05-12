from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_map_memory_description():
    """Launch map memory node."""
    map_memory_pkg_prefix = get_package_share_directory('map_memory')
    map_memory_param_file = os.path.join(
        map_memory_pkg_prefix, 'config', 'params.yaml')

    map_memory_param = DeclareLaunchArgument(
        'map_memory_param_file',
        default_value=map_memory_param_file,
        description='Path to config file for map memory node'
    )

    map_memory_node = Node(
        package='map_memory',
        executable='map_memory_node',
        parameters=[LaunchConfiguration('map_memory_param_file')],
    )

    return LaunchDescription([
        map_memory_param,
        map_memory_node
    ])
