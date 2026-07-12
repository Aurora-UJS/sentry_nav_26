#!/usr/bin/env python3
"""记录自转下的幽灵占据格数量: /sdf_map/occupancy 中距机器人 < R 的格子数.

机器人在开阔区自转时, 半径 R 内本应无占据格 (真墙更远), 计数即幽灵数量.
输出 CSV: t, phantom_count, total_count
用法: occ_phantom_recorder.py <out.csv> <duration_s> [R=1.2]
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class Recorder(Node):
    def __init__(self, out_path, duration, radius):
        super().__init__("occ_phantom_recorder")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.out = open(out_path, "w")
        self.out.write("t,phantom_count,total_count\n")
        self.duration = duration
        self.radius = radius
        self.t0 = None
        self.pos = None
        self.done = False
        self.create_subscription(Odometry, "/Odometry", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/sdf_map/occupancy", self.on_occ, 10)

    def on_odom(self, msg):
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def on_occ(self, msg):
        if self.pos is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        px, py = self.pos
        r2 = self.radius**2
        total = 0
        phantom = 0
        for x, y, _z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            total += 1
            if (x - px) ** 2 + (y - py) ** 2 < r2:
                phantom += 1
        self.out.write(f"{t:.3f},{phantom},{total}\n")
        self.out.flush()
        if t >= self.duration:
            self.done = True


def main():
    out_path = sys.argv[1]
    duration = float(sys.argv[2])
    radius = float(sys.argv[3]) if len(sys.argv) > 3 else 1.2
    rclpy.init()
    node = Recorder(out_path, duration, radius)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.out.close()
    print(f"done: {out_path}")


if __name__ == "__main__":
    main()
