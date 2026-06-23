"""sentry_global_planner launch"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    map_yaml = LaunchConfiguration("map_yaml", default="")

    # 可通行性标注层 .trav.yaml 路径。默认空 = 禁用 (独立启动本 launch 不依赖其它包)。
    # 整机/仿真由上层 sentry_bringup 注入 (它依赖本包; 反向引用其 share 会成依赖环)。
    trav_yaml = LaunchConfiguration("trav_yaml", default="")

    params_file = os.path.join(
        get_package_share_directory("sentry_global_planner"),
        "config",
        "global_planner_params.yaml",
    )

    global_planner_node = Node(
        package="sentry_global_planner",
        executable="sentry_global_planner_node",
        name="sentry_global_planner",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
            {"global_map.yaml_path": map_yaml},
            {"traversability.yaml_path": trav_yaml},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("map_yaml", default_value=""),
            DeclareLaunchArgument("trav_yaml", default_value=""),
            global_planner_node,
        ]
    )
