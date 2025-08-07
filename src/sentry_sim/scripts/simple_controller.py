#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.get_logger().info("Simple controller started")

    def move_forward(self, duration=2.0, speed=0.5):
        """前进"""
        self.get_logger().info(f"Moving forward for {duration} seconds at {speed} m/s")
        twist = Twist()
        twist.linear.x = speed

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(twist)
            time.sleep(0.1)

        self.stop()

    def move_backward(self, duration=2.0, speed=0.5):
        """后退"""
        self.get_logger().info(f"Moving backward for {duration} seconds at {speed} m/s")
        twist = Twist()
        twist.linear.x = -speed

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(twist)
            time.sleep(0.1)

        self.stop()

    def move_left(self, duration=2.0, speed=0.5):
        """左移"""
        self.get_logger().info(f"Moving left for {duration} seconds at {speed} m/s")
        twist = Twist()
        twist.linear.y = speed

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(twist)
            time.sleep(0.1)

        self.stop()

    def move_right(self, duration=2.0, speed=0.5):
        """右移"""
        self.get_logger().info(f"Moving right for {duration} seconds at {speed} m/s")
        twist = Twist()
        twist.linear.y = -speed

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(twist)
            time.sleep(0.1)

        self.stop()

    def rotate_left(self, duration=2.0, speed=0.5):
        """左转"""
        self.get_logger().info(f"Rotating left for {duration} seconds at {speed} rad/s")
        twist = Twist()
        twist.angular.z = speed

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(twist)
            time.sleep(0.1)

        self.stop()

    def rotate_right(self, duration=2.0, speed=0.5):
        """右转"""
        self.get_logger().info(
            f"Rotating right for {duration} seconds at {speed} rad/s"
        )
        twist = Twist()
        twist.angular.z = -speed

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(twist)
            time.sleep(0.1)

        self.stop()

    def move_diagonal(self, duration=2.0, vx=0.3, vy=0.3):
        """斜向移动"""
        self.get_logger().info(
            f"Moving diagonal for {duration} seconds at vx={vx}, vy={vy}"
        )
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(twist)
            time.sleep(0.1)

        self.stop()

    def stop(self):
        """停止"""
        self.get_logger().info("Stopping")
        twist = Twist()
        self.publisher.publish(twist)
        time.sleep(0.5)

    def demo_sequence(self):
        """演示序列"""
        self.get_logger().info("Starting demo sequence")

        # 前进
        self.move_forward(2.0, 0.5)
        time.sleep(1)

        # 左移
        self.move_left(2.0, 0.5)
        time.sleep(1)

        # 后退
        self.move_backward(2.0, 0.5)
        time.sleep(1)

        # 右移
        self.move_right(2.0, 0.5)
        time.sleep(1)

        # 左转
        self.rotate_left(2.0, 0.5)
        time.sleep(1)

        # 右转
        self.rotate_right(2.0, 0.5)
        time.sleep(1)

        # 斜向移动
        self.move_diagonal(2.0, 0.3, 0.3)
        time.sleep(1)

        self.get_logger().info("Demo sequence completed")


def main(args=None):
    rclpy.init(args=args)
    controller = SimpleController()

    try:
        # 运行演示序列
        controller.demo_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
