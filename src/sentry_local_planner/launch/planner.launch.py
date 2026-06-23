"""sentry_local_planner launch: 规划节点"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    planner_params = os.path.join(
        get_package_share_directory("sentry_local_planner"),
        "config",
        "planner_params.yaml",
    )

    # 静态可通行性标注层 .trav.yaml 路径。默认空 = 禁用 (独立启动本 launch 不依赖其它包)。
    # 整机/仿真由上层 sentry_bringup 注入实际路径 — bringup 是地图路径的唯一来源, 且 bringup
    # 依赖本包; 反向引用其 share 目录会造成依赖环。可用 trav_yaml:=<路径> 覆盖。
    trav_yaml = LaunchConfiguration("trav_yaml", default="")

    planner_node = Node(
        package="sentry_local_planner",
        executable="sentry_local_planner_node",
        name="sentry_local_planner",
        output="screen",
        parameters=[
            planner_params,
            {"use_sim_time": use_sim_time},
            {"traversability.yaml_path": trav_yaml},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("trav_yaml", default_value=""),
            planner_node,
        ]
    )
