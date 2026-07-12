#!/usr/bin/env python3
"""量化轨迹跟踪质量: 横向误差 / 路径抖动 / 速度跟踪.

订阅 /Odometry, /planning/trajectory, /cmd_vel_world, 发目标点并记录全程.
输出 CSV:
  <out>_track.csv: t, cross_track_err, v_actual, v_cmd
  <out>_churn.csv: t, plan_seq, churn(相邻规划在机器人前方 2m 弧段的平均偏移)
用法: tracking_recorder.py <out_prefix> <goal_x> <goal_y> <duration_s>
"""
import math
import sys

import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path


def nearest_dist(poly, p):
    """点到折线的最近距离."""
    best = float("inf")
    for i in range(len(poly) - 1):
        ax, ay = poly[i]
        bx, by = poly[i + 1]
        vx, vy = bx - ax, by - ay
        L2 = vx * vx + vy * vy
        if L2 < 1e-12:
            d = math.hypot(p[0] - ax, p[1] - ay)
        else:
            t = max(0.0, min(1.0, ((p[0] - ax) * vx + (p[1] - ay) * vy) / L2))
            d = math.hypot(p[0] - (ax + t * vx), p[1] - (ay + t * vy))
        best = min(best, d)
    return best


def arc_from(poly, p, arc_len):
    """从折线上离 p 最近的点起, 取前方 arc_len 米的采样点."""
    # 找最近段
    besti, bestd = 0, float("inf")
    for i, q in enumerate(poly):
        d = math.hypot(q[0] - p[0], q[1] - p[1])
        if d < bestd:
            bestd, besti = d, i
    out = [poly[besti]]
    acc = 0.0
    for i in range(besti + 1, len(poly)):
        acc += math.hypot(poly[i][0] - poly[i - 1][0], poly[i][1] - poly[i - 1][1])
        out.append(poly[i])
        if acc >= arc_len:
            break
    return out


def main():
    prefix, gx, gy, dur = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
    rclpy.init()
    node = rclpy.create_node(
        "tracking_recorder",
        parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)],
    )
    ftrack = open(prefix + "_track.csv", "w")
    ftrack.write("t,cross_track_err,v_actual,v_cmd\n")
    fchurn = open(prefix + "_churn.csv", "w")
    fchurn.write("t,plan_seq,churn\n")

    state = {"pos": None, "vel": 0.0, "last_pos": None, "last_t": None,
             "traj": None, "prev_traj": None, "seq": 0, "cmd": 0.0, "t0": None,
             "reached_t": None}

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def on_odom(msg):
        t = now()
        if state["t0"] is None:
            state["t0"] = t
        p = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if state["last_pos"] is not None and state["last_t"] is not None:
            dt = t - state["last_t"]
            if 1e-4 < dt < 0.5:
                v = math.hypot(p[0] - state["last_pos"][0], p[1] - state["last_pos"][1]) / dt
                state["vel"] = 0.7 * state["vel"] + 0.3 * v
        state["last_pos"] = p
        state["last_t"] = t
        state["pos"] = p
        if state["traj"] and len(state["traj"]) > 1:
            err = nearest_dist(state["traj"], p)
            ftrack.write(f"{t - state['t0']:.3f},{err:.4f},{state['vel']:.3f},{state['cmd']:.3f}\n")
        if state["reached_t"] is None and math.hypot(p[0] - gx, p[1] - gy) < 1.0:
            state["reached_t"] = t - state["t0"]

    def on_traj(msg):
        t = now()
        if state["t0"] is None:
            state["t0"] = t
        poly = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(poly) < 2:
            return
        state["seq"] += 1
        if state["traj"] and state["pos"]:
            # 抖动: 新旧规划在机器人前方 2m 弧段上的平均偏移
            arc = arc_from(state["traj"], state["pos"], 2.0)
            if len(arc) >= 2:
                ds = [nearest_dist(poly, q) for q in arc]
                churn = sum(ds) / len(ds)
                fchurn.write(f"{t - state['t0']:.3f},{state['seq']},{churn:.4f}\n")
        state["traj"] = poly

    def on_cmd(msg):
        state["cmd"] = math.hypot(msg.linear.x, msg.linear.y)

    node.create_subscription(Odometry, "/Odometry", on_odom, qos_profile_sensor_data)
    node.create_subscription(Path, "/planning/trajectory", on_traj, 10)
    node.create_subscription(Twist, "/cmd_vel_world", on_cmd, qos_profile_sensor_data)
    pub = node.create_publisher(PoseStamped, "/goal_pose", 10)

    g = PoseStamped()
    g.header.frame_id = "odom"
    g.pose.position.x = gx
    g.pose.position.y = gy
    g.pose.orientation.w = 1.0

    import time
    end = time.time() + dur
    sent = 0
    while time.time() < end:
        if sent < 3:
            pub.publish(g)
            sent += 1
        rclpy.spin_once(node, timeout_sec=0.2)
    ftrack.close()
    fchurn.close()
    print(f"done reached={state['reached_t']} plans={state['seq']}")


if __name__ == "__main__":
    main()
