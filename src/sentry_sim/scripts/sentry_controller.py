#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import math
import tf2_ros
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class SentryController(Node):
    def __init__(self):
        super().__init__("sentry_controller")

        # 订阅底盘移动指令
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

        # 订阅云台控制指令
        self.turret_sub = self.create_subscription(
            Float64, "/turret_cmd", self.turret_callback, 10
        )

        self.barrel_sub = self.create_subscription(
            Float64, "/barrel_cmd", self.barrel_callback, 10
        )

        # 发布各个轮子的速度
        self.front_left_pub = self.create_publisher(
            Float64, "/front_left_wheel/cmd_vel", 10
        )
        self.front_right_pub = self.create_publisher(
            Float64, "/front_right_wheel/cmd_vel", 10
        )
        self.back_left_pub = self.create_publisher(
            Float64, "/back_left_wheel/cmd_vel", 10
        )
        self.back_right_pub = self.create_publisher(
            Float64, "/back_right_wheel/cmd_vel", 10
        )

        # 发布云台控制
        self.turret_pub = self.create_publisher(Float64, "/turret/cmd_pos", 10)
        self.barrel_pub = self.create_publisher(Float64, "/barrel/cmd_pos", 10)

        # 发布里程计
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # 机器人参数
        self.wheel_radius = 0.08  # 轮子半径
        self.wheel_base = 0.3  # 前后轮距离
        self.wheel_track = 0.4  # 左右轮距离

        # 里程计状态
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # 定时器发布里程计
        self.odom_timer = self.create_timer(0.01, self.publish_odom)

        # 当前速度
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0

        # 云台位置
        self.turret_angle = 0.0
        self.barrel_angle = 0.0

        self.get_logger().info("Sentry robot controller started")
        self.get_logger().info("Control:")
        self.get_logger().info("  - Chassis: /cmd_vel (geometry_msgs/Twist)")
        self.get_logger().info("  - Turret: /turret_cmd (std_msgs/Float64)")
        self.get_logger().info("  - Barrel: /barrel_cmd (std_msgs/Float64)")

    def cmd_vel_callback(self, msg):
        """
        麦克纳姆轮运动学方程（修正版）：
        通过轮子差速实现旋转
        """
        self.current_vx = msg.linear.x
        self.current_vy = msg.linear.y
        self.current_wz = msg.angular.z

        # 计算各轮子的线速度
        lx = self.wheel_base / 2  # 0.15
        ly = self.wheel_track / 2  # 0.2

        # 麦克纳姆轮运动学公式 - 确保旋转是通过轮子差速实现的
        front_left_vel = (
            self.current_vx - self.current_vy - self.current_wz * (lx + ly)
        ) / self.wheel_radius
        front_right_vel = (
            self.current_vx + self.current_vy + self.current_wz * (lx + ly)
        ) / self.wheel_radius
        back_left_vel = (
            self.current_vx + self.current_vy - self.current_wz * (lx + ly)
        ) / self.wheel_radius
        back_right_vel = (
            self.current_vx - self.current_vy + self.current_wz * (lx + ly)
        ) / self.wheel_radius

        # 发布轮子速度
        self.front_left_pub.publish(Float64(data=front_left_vel))
        self.front_right_pub.publish(Float64(data=front_right_vel))
        self.back_left_pub.publish(Float64(data=back_left_vel))
        self.back_right_pub.publish(Float64(data=back_right_vel))

        # 调试输出
        if (
            abs(self.current_vx) > 0.01
            or abs(self.current_vy) > 0.01
            or abs(self.current_wz) > 0.01
        ):
            self.get_logger().info(
                f"Chassis CMD: vx={self.current_vx:.2f}, vy={self.current_vy:.2f}, wz={self.current_wz:.2f}"
            )
            self.get_logger().info(
                f"Wheels: FL={front_left_vel:.2f}, FR={front_right_vel:.2f}, BL={back_left_vel:.2f}, BR={back_right_vel:.2f}"
            )

    def turret_callback(self, msg):
        """云台水平旋转控制"""
        self.turret_angle = msg.data
        self.turret_pub.publish(Float64(data=self.turret_angle))
        self.get_logger().info(f"Turret angle: {math.degrees(self.turret_angle):.1f}°")

    def barrel_callback(self, msg):
        """炮管俯仰控制"""
        self.barrel_angle = msg.data
        self.barrel_pub.publish(Float64(data=self.barrel_angle))
        self.get_logger().info(f"Barrel angle: {math.degrees(self.barrel_angle):.1f}°")

    def publish_odom(self):
        """发布里程计信息"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # 更新位置（通过轮子差速计算）
        delta_x = (
            self.current_vx * math.cos(self.theta)
            - self.current_vy * math.sin(self.theta)
        ) * dt
        delta_y = (
            self.current_vx * math.sin(self.theta)
            + self.current_vy * math.cos(self.theta)
        ) * dt
        delta_theta = self.current_wz * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # 创建四元数
        quat_z = math.sin(self.theta / 2.0)
        quat_w = math.cos(self.theta / 2.0)

        # 发布TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = quat_z
        t.transform.rotation.w = quat_w

        self.tf_broadcaster.sendTransform(t)

        # 发布里程计
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # 位置
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = quat_z
        odom.pose.pose.orientation.w = quat_w

        # 速度
        odom.twist.twist.linear.x = self.current_vx
        odom.twist.twist.linear.y = self.current_vy
        odom.twist.twist.angular.z = self.current_wz

        self.odom_pub.publish(odom)
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = SentryController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
