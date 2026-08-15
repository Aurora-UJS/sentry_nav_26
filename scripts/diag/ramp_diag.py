#!/usr/bin/env python3
"""坡道卡住诊断：采集 4s 的位姿/倾角、轨迹首末点、cmd_vel，一次判定卡因。

用法（容器内，卡住时跑）：
    python3 scripts/diag/ramp_diag.py

判定：
    tilt_deg   ~0        → 坡道模式未触发（排除 slope 模式问题）
    tilt_deg   >7        → slope 对齐模式激活中
    TRAJ end vs ROBOT    → 轨迹终点在车前方多远：轨迹是否延伸上坡
    CMD lin    >0 持续   → 控制器在命令往前/上坡（车不动=物理卡）
    CMD lin    ~0/反向   → 控制层没让它爬（规划/对齐把车按住）
"""
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist


class Diag(Node):
    def __init__(self):
        super().__init__("ramp_diag")
        self.pos = None
        self.tilt = None
        self.yaw = None
        self.cmds = []
        self.traj = {}
        self.create_subscription(Odometry, "/Odometry", self.on_odom, 10)
        self.create_subscription(Twist, "/cmd_vel_world", self.on_cmd, 10)
        self.create_subscription(Path, "/planning/trajectory", self.on_traj, 10)

    def on_odom(self, m):
        p = m.pose.pose.position
        self.pos = (p.x, p.y, p.z)
        q = m.pose.pose.orientation
        r22 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.tilt = math.acos(max(-1.0, min(1.0, r22))) * 180.0 / math.pi
        self.yaw = 2.0 * math.atan2(q.z, q.w) * 180.0 / math.pi

    def on_cmd(self, m):
        self.cmds.append((m.linear.x, m.linear.y, m.angular.z))

    def on_traj(self, m):
        if m.poses:
            s = m.poses[0].pose.position
            e = m.poses[-1].pose.position
            self.traj = dict(n=len(m.poses),
                             start=(round(s.x, 3), round(s.y, 3)),
                             end=(round(e.x, 3), round(e.y, 3)))


def main():
    rclpy.init()
    n = Diag()
    t0 = time.time()
    while time.time() - t0 < 4:
        rclpy.spin_once(n, timeout_sec=0.1)

    if n.pos:
        p = n.pos
        print("ROBOT pos(x,y,z)=(%.3f, %.3f, %.3f) tilt=%.2fdeg yaw=%.1fdeg"
              % (p[0], p[1], p[2], n.tilt or 0.0, n.yaw or 0.0))
        if n.traj:
            s, e = n.traj["start"], n.traj["end"]
            d_end = math.hypot(e[0] - p[0], e[1] - p[1])
            d_start = math.hypot(s[0] - p[0], s[1] - p[1])
            print("TRAJ n=%d start=%s end=%s |dist(robot->start)|=%.2fm |dist(robot->end)|=%.2fm"
                  % (n.traj["n"], s, e, d_start, d_end))
        else:
            print("TRAJ none")
        if n.cmds:
            print("CMD last5 (lin.x, lin.y, wz):")
            for c in n.cmds[-5:]:
                print("    (%.2f, %.2f, %.2f)" % c)
            spd = [abs(c[0]) + abs(c[1]) for c in n.cmds]
            print("CMD avg |lin| = %.2f m/s over %d samples" % (sum(spd) / len(spd), len(spd)))
        else:
            print("CMD none")
    else:
        print("NO ODOM RECEIVED (4s) — 检查 zenoh router / bringup 是否在跑")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
