"""sentry_planner launch: 规划节点"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    planner_params = os.path.join(
        get_package_share_directory("sentry_planner"),
        "config",
        "planner_params.yaml",
    )

    planner_node = Node(
        package="sentry_planner",
        executable="sentry_planner_node",
        name="sentry_planner",
        output="screen",
        parameters=[planner_params, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            planner_node,
        ]
    )
