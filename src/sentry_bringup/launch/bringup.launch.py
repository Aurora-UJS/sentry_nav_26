from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    simulation_launch = os.path.join(
        get_package_share_directory("sentry_sim"), "launch", "simulation.launch.py"
    )
    localization_launch = os.path.join(
        get_package_share_directory("sentry_localization"),
        "launch",
        "localization.launch.py",
    )
    navigation_launch = os.path.join(
        get_package_share_directory("sentry_nav"), "launch", "navigation.launch.py"
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(simulation_launch)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localization_launch)
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(navigation_launch)),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(
                        get_package_share_directory("sentry_bringup"),
                        "config",
                        "nav2_rviz_config.rviz",
                    ),
                ],
            ),
        ]
    )
