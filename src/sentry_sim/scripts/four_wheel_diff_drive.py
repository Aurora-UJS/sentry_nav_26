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

class FourWheelDiffDrive(Node):
    def __init__(self):
        super().__init__('four_wheel_diff_drive')
        
        # 订阅cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 发布各个轮子的速度
        self.front_left_pub = self.create_publisher(Float64, '/front_left_wheel/cmd_vel', 10)
        self.front_right_pub = self.create_publisher(Float64, '/front_right_wheel/cmd_vel', 10)
        self.back_left_pub = self.create_publisher(Float64, '/back_left_wheel/cmd_vel', 10)
        self.back_right_pub = self.create_publisher(Float64, '/back_right_wheel/cmd_vel', 10)
        
        # 发布里程计
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 机器人参数
        self.wheel_radius = 0.08  # 轮子半径
        self.wheel_separation = 0.4  # 左右轮距离
        self.wheel_base = 0.3  # 前后轮距离
        
        # 里程计状态
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # 定时器发布里程计
        self.odom_timer = self.create_timer(0.01, self.publish_odom)
        
        # 当前速度
        self.current_vx = 0.0
        self.current_wz = 0.0
        
        self.get_logger().info('Four wheel diff drive controller started')
    
    def cmd_vel_callback(self, msg):
        """
        将cmd_vel转换为四个轮子的速度
        四轮差速驱动：前后轮同步，左右轮差速
        """
        self.current_vx = msg.linear.x
        self.current_wz = msg.angular.z
        
        # 计算左右轮的线速度
        left_wheel_speed = (self.current_vx - self.current_wz * self.wheel_separation / 2) / self.wheel_radius
        right_wheel_speed = (self.current_vx + self.current_wz * self.wheel_separation / 2) / self.wheel_radius
        
        # 四个轮子：前后轮同步
        self.front_left_pub.publish(Float64(data=left_wheel_speed))
        self.front_right_pub.publish(Float64(data=right_wheel_speed))
        self.back_left_pub.publish(Float64(data=left_wheel_speed))
        self.back_right_pub.publish(Float64(data=right_wheel_speed))
        
        self.get_logger().debug(f'CMD: vx={self.current_vx:.2f}, wz={self.current_wz:.2f}')
        self.get_logger().debug(f'Wheels: L={left_wheel_speed:.2f}, R={right_wheel_speed:.2f}')
    
    def publish_odom(self):
        """发布里程计信息"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # 更新位置
        delta_x = self.current_vx * math.cos(self.theta) * dt
        delta_y = self.current_vx * math.sin(self.theta) * dt
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
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
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
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
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
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.current_wz
        
        # 协方差矩阵
        odom.pose.covariance = [0.1, 0, 0, 0, 0, 0,
                               0, 0.1, 0, 0, 0, 0,
                               0, 0, 1e6, 0, 0, 0,
                               0, 0, 0, 1e6, 0, 0,
                               0, 0, 0, 0, 1e6, 0,
                               0, 0, 0, 0, 0, 0.1]
        
        odom.twist.covariance = [0.1, 0, 0, 0, 0, 0,
                               0, 0.1, 0, 0, 0, 0,
                               0, 0, 1e6, 0, 0, 0,
                               0, 0, 0, 1e6, 0, 0,
                               0, 0, 0, 0, 1e6, 0,
                               0, 0, 0, 0, 0, 0.1]
        
        self.odom_pub.publish(odom)
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = FourWheelDiffDrive()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
