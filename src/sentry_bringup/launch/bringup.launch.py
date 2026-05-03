"""
sentry_bringup 主 launch 文件

当前阶段: 只启动仿真 + RViz2
TODO: 后续阶段逐步加入 localization、navigation
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_teleop = LaunchConfiguration("start_teleop")

    # 使用 sim_test launch 作为默认入口
    sim_test_launch = os.path.join(
        get_package_share_directory("sentry_bringup"),
        "launch",
        "sim_test.launch.py",
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_test_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "start_teleop": start_teleop,
                }.items(),
            ),
        ]
    )
