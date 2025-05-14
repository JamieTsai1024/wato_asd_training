from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """Launch control node."""
    control_pkg_prefix = get_package_share_directory('control')
    control_param_file = os.path.join(
        control_pkg_prefix, 'config', 'params.yaml')

    control_param = DeclareLaunchArgument(
        'control_param_file',
        default_value=control_param_file,
        description='Path to config file for control node'
    )

    control_node = Node(
        package='control',
        executable='control_node',
        parameters=[LaunchConfiguration('control_param_file')],
    )

    return LaunchDescription([
        control_param,
        control_node
    ])
