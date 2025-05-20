from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """Launch planner node."""
    planner_pkg_prefix = get_package_share_directory('planner')
    planner_param_file = os.path.join(
        planner_pkg_prefix, 'config', 'params.yaml')

    planner_param = DeclareLaunchArgument(
        'planner_param_file',
        default_value=planner_param_file,
        description='Path to config file for planner node'
    )

    planner_node = Node(
        package='planner',
        executable='planner_node',
        parameters=[LaunchConfiguration('planner_param_file')],
    )

    return LaunchDescription([
        planner_param,
        planner_node
    ])
