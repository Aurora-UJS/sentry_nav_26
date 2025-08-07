from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    world = PathJoinSubstitution(
        [FindPackageShare("sentry_sim"), "worlds", "my_world.world"]
    )

    robot_sdf = PathJoinSubstitution(
        [FindPackageShare("sentry_sim"), "models", "my_robot", "my_robot.sdf"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="world",
                default_value=world,
                description="Path to the Gazebo world file",
            ),
            DeclareLaunchArgument(
                name="robot_sdf",
                default_value=robot_sdf,
                description="Path to the robot SDF file",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                    )
                ),
                launch_arguments={
                    "gz_args": ["-r ", LaunchConfiguration("world")]
                }.items(),
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-file",
                    LaunchConfiguration("robot_sdf"),
                    "-name",
                    "my_robot",
                    "-x",
                    "0",
                    "-y",
                    "0",
                    "-z",
                    "0.1",
                ],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
                    "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
                    "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                    "/turret_cmd@std_msgs/msg/Float64@gz.msgs.Double",
                    "/barrel_cmd@std_msgs/msg/Float64@gz.msgs.Double",
                    "/front_left_wheel/cmd_vel@std_msgs/msg/Float64@gz.msgs.Double",
                    "/front_right_wheel/cmd_vel@std_msgs/msg/Float64@gz.msgs.Double",
                    "/back_left_wheel/cmd_vel@std_msgs/msg/Float64@gz.msgs.Double",
                    "/back_right_wheel/cmd_vel@std_msgs/msg/Float64@gz.msgs.Double",
                    "/turret/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double",
                    "/barrel/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double",
                ],
                output="screen",
            ),
            Node(
                package="sentry_sim",
                executable="sentry_controller.py",
                name="sentry_controller",
                output="screen",
            ),
        ]
    )
