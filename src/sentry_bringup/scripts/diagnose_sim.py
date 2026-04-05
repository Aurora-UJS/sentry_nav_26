"""
诊断脚本: 检查 small_point_lio 仿真链路的数据连通性

用法:
  source install/setup.bash
  ros2 launch sentry_bringup sim_test.launch.py  # 先启动仿真
  # 另一个终端:
  source install/setup.bash
  python3 src/sentry_bringup/scripts/diagnose_sim.py
"""

import subprocess
import time
import sys


def run(cmd, timeout=5):
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=timeout
        )
        return result.stdout.strip()
    except subprocess.TimeoutExpired:
        return "[TIMEOUT]"


def main():
    print("=" * 60)
    print("Sentry Nav 仿真链路诊断")
    print("=" * 60)

    # 1. 检查 RMW
    rmw = run("echo $RMW_IMPLEMENTATION")
    print(f"\n[RMW] 中间件: {rmw or '默认'}")

    # 2. 列出所有 topic
    print("\n[TOPICS] 活跃 topic 列表:")
    topics = run("ros2 topic list", timeout=10)
    print(topics)

    # 3. 检查关键 topic 是否有数据
    print("\n" + "-" * 40)
    key_topics = ["/livox/lidar", "/livox/imu", "/Odometry", "/cloud_registered", "/cmd_vel"]
    for topic in key_topics:
        if topic in (topics or ""):
            hz = run(f"ros2 topic hz {topic} --window 5", timeout=8)
            info = run(f"ros2 topic info {topic} -v", timeout=5)
            pub_count = info.count("Publisher count")
            sub_count = info.count("Subscriber count")
            print(f"\n[{topic}]")
            print(f"  info: {info[:300]}")
            if hz and hz != "[TIMEOUT]":
                print(f"  hz: {hz[:200]}")
            else:
                print("  hz: 无数据 (可能没有发布)")
        else:
            print(f"\n[{topic}] 不存在!")

    # 4. 检查 TF
    print("\n" + "-" * 40)
    print("\n[TF] frames:")
    frames = run("ros2 run tf2_ros tf2_echo odom base_link", timeout=5)
    if "Could not" in frames or "Failure" in frames or not frames:
        print("  odom -> base_link: 不存在!")
    else:
        print(f"  odom -> base_link: {frames[:200]}")

    frames2 = run("ros2 run tf2_ros tf2_echo base_link livox_lidar", timeout=5)
    if "Could not" in frames2 or not frames2:
        print("  base_link -> livox_lidar: 不存在!")
    else:
        print("  base_link -> livox_lidar: 存在")

    # 5. 检查 small_point_lio 节点状态
    print("\n" + "-" * 40)
    nodes = run("ros2 node list", timeout=5)
    print(f"\n[NODES] 活跃节点:\n{nodes}")

    if "/small_point_lio" in (nodes or ""):
        params = run("ros2 param list /small_point_lio", timeout=5)
        print(f"\n[small_point_lio params]: {params[:300]}")

    print("\n" + "=" * 60)
    print("诊断完成")


if __name__ == "__main__":
    main()
