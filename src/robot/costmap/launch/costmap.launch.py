from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """Launch costmap node."""
    costmap_pkg_prefix = get_package_share_directory('costmap')
    costmap_param_file = os.path.join(
        costmap_pkg_prefix, 'config', 'params.yaml')

    costmap_param = DeclareLaunchArgument(
        'costmap_param_file',
        default_value=costmap_param_file,
        description='Path to config file for costmap node'
    )

    costmap_node = Node(
        package='costmap',
        executable='costmap_node',
        parameters=[LaunchConfiguration('costmap_param_file')],
    )

    return LaunchDescription([
        costmap_param,
        costmap_node
    ])
