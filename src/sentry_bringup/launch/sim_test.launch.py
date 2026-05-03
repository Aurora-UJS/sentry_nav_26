"""
阶段1集成 launch: RMUC 仿真 + Small Point-LIO + teleop + RViz2

数据流:
  rm_sim_26 (Gazebo)  →  /livox/lidar, /livox/imu, /cmd_vel
                               ↓
  small_point_lio     →  /odom, /cloud_registered
                               ↓
  RViz2               ←  可视化点云、位姿、TF
  teleop_twist_keyboard →  /cmd_vel 手动遥控
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    start_teleop = LaunchConfiguration("start_teleop", default="true")

    # === 1. RMUC 2025 仿真环境 ===
    rmuc_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rm_sim_26"),
                "launch",
                "rmuc_2025_sim.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # === 2. Small Point-LIO (仿真模式) ===
    # 延迟启动，等 Gazebo 和传感器 bridge 就绪
    lio_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("small_point_lio"),
                        "launch",
                        "run_sim.launch.py",
                    )
                ),
                launch_arguments={"use_sim_time": "true"}.items(),
            )
        ],
    )

    # === 3. RViz2 可视化 ===
    rviz_config = os.path.join(
        get_package_share_directory("sentry_bringup"),
        "config",
        "sim_rviz.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # === 4. 静态 map → odom TF (占位，后续由全局定位替换) ===
    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "map",
            "--child-frame-id", "odom",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # === 5. Teleop (在新终端中启动) ===
    teleop_node = ExecuteProcess(
        cmd=[
            "gnome-terminal",
            "--",
            "bash",
            "-c",
            "source /opt/ros/jazzy/setup.bash && "
            "source " + os.path.expanduser("~/sentry_nav_26/install/setup.bash") + " && "
            "ros2 run teleop_twist_keyboard teleop_twist_keyboard "
            "--ros-args -p use_sim_time:=true; exec bash",
        ],
        condition=IfCondition(start_teleop),
        output="screen",
    )

    # === 6. Sentry Planner (延迟10秒，等LIO初始化完成) ===
    planner_launch = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("sentry_local_planner"),
                        "launch",
                        "planner.launch.py",
                    )
                ),
                launch_arguments={"use_sim_time": "true"}.items(),
            )
        ],
    )

    # === 7. Global Planner (延迟12秒，与 local planner 同时) ===
    map_yaml_path = os.path.join(
        get_package_share_directory("sentry_bringup"),
        "map",
        "rmuc_2025.yaml",
    )
    global_planner_launch = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("sentry_global_planner"),
                        "launch",
                        "global_planner.launch.py",
                    )
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "map_yaml": map_yaml_path,
                }.items(),
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use sim time"
            ),
            DeclareLaunchArgument(
                "start_teleop",
                default_value="true",
                description="Start teleop_twist_keyboard in a new terminal",
            ),
            # 仿真环境
            rmuc_sim_launch,
            # LIO (延迟6秒，等传感器就绪)
            lio_launch,
            # map → odom 静态TF
            map_to_odom_tf,
            # RViz
            rviz_node,
            # Teleop (可选，在新终端窗口)
            teleop_node,
            # Planner (延迟12秒)
            planner_launch,
            # Global Planner (延迟12秒)
            global_planner_launch,
        ]
    )
