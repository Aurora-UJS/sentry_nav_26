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

    # 静态先验层：与全局规划器共用同一张标注地图（确定性不可通行基准）
    try:
        static_map_path = os.path.join(
            get_package_share_directory("sentry_bringup"), "map", "rmuc_2025.yaml"
        )
    except Exception:
        static_map_path = ""

    planner_node = Node(
        package="sentry_local_planner",
        executable="sentry_local_planner_node",
        name="sentry_local_planner",
        output="screen",
        parameters=[
            planner_params,
            {"use_sim_time": use_sim_time, "sdf_map.static_map_path": static_map_path},
        ],
    )

    # 仿真侧"底盘固件": /cmd_vel_world → 陀螺 yaw 旋转 → /cmd_vel
    # 真车部署时删除本节点，由电控 MCU 实现同一逻辑
    chassis_cmd_node = Node(
        package="sentry_local_planner",
        executable="chassis_cmd_node",
        name="chassis_cmd",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            planner_node,
            chassis_cmd_node,
        ]
    )
