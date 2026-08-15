#!/usr/bin/env python3
"""集成测试输入源：固定位姿 odometry(50Hz) + 一次性 goal。

验证 sentry_local_planner 在静态先验地图(无 LIO/点云)下的规划+控制链路：
- /Odometry: 固定位姿(0,0)、零速，墙钟时间戳
- /goal_pose: 3 秒后发一次 (3,0)
- 订阅 /cmd_vel_world，每 50 帧打一行，确认 50Hz 控制回路在动
"""
import threading
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist

START = (0.0, 0.0)
GOAL = (3.0, 0.0)


class Pub(Node):
    def __init__(self):
        super().__init__("pub_odom_goal")
        self.odom_pub = self.create_publisher(Odometry, "/Odometry", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.create_subscription(Twist, "/cmd_vel_world", self.on_cmd, 10)
        self.cmd_n = 0
        self.create_timer(0.02, self.pub_odom)

    def pub_odom(self):
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "odom"
        m.child_frame_id = "base_link"
        m.pose.pose.position.x = float(START[0])
        m.pose.pose.position.y = float(START[1])
        m.pose.pose.orientation.w = 1.0
        # twist 留空 → 节点退回位姿差分（固定位姿 → 速度 0）
        self.odom_pub.publish(m)

    def send_goal(self):
        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = "odom"
        g.pose.position.x = float(GOAL[0])
        g.pose.position.y = float(GOAL[1])
        g.pose.orientation.w = 1.0
        self.goal_pub.publish(g)
        self.get_logger().info(f"GOAL sent ({GOAL[0]},{GOAL[1]})")

    def on_cmd(self, msg):
        self.cmd_n += 1
        if self.cmd_n % 50 == 1:
            self.get_logger().info(
                f"cmd_vel_world #{self.cmd_n}: "
                f"lin=({msg.linear.x:.2f},{msg.linear.y:.2f}) ang={msg.angular.z:.2f}"
            )


def main():
    rclpy.init()
    n = Pub()

    def delayed_goal():
        time.sleep(3.0)
        n.send_goal()

    threading.Thread(target=delayed_goal, daemon=True).start()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
