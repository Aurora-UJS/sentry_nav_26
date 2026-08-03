"""真车串口桥。仿真不需要本节点（chassis_cmd_node 扮演 MCU）。

真车 bringup 时 include 本文件，并确保仿真专用的 chassis_cmd_node 不启动。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('sentry_serial'), 'config', 'serial_params.yaml')

    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyACM0')

    return LaunchDescription([
        port_arg,
        Node(
            package='sentry_serial',
            executable='sentry_serial_node',
            name='sentry_serial',
            output='screen',
            parameters=[params, {'port': LaunchConfiguration('port')}],
        ),
    ])
