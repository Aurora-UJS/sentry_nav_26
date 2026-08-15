#!/usr/bin/env bash
# 无 Gazebo/LIO 的规划链路集成测试：静态先验地图 + 合成 odometry + goal。
# 验证 P0(多线程 executor) + P1(A*/MINCO) 在真实节点运行时无崩溃/无竞态。
set -e
source /opt/ros/jazzy/setup.bash
source /workspace/sentry_nav_26/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # 单容器多节点用本地 DDS，不依赖 zenoh 路由

MAP=/workspace/sentry_nav_26/src/sentry_bringup/map/rmuc_2025.yaml
CFG=/workspace/sentry_nav_26/src/sentry_local_planner/config/planner_params.yaml
OVR=/workspace/sentry_nav_26/scripts/diag/integration_map_override.yaml
DUR=${1:-40}

echo "=== start global planner ==="
ros2 run sentry_global_planner sentry_global_planner_node --ros-args \
  -r __node:=sentry_global_planner \
  -p use_sim_time:=false \
  -p global_map.yaml_path:="$MAP" \
  > /tmp/gp.log 2>&1 &
GP_PID=$!

echo "=== start local planner (static map, no cloud) ==="
ros2 run sentry_local_planner sentry_local_planner_node --ros-args \
  -r __node:=sentry_local_planner \
  -p use_sim_time:=false \
  --params-file "$CFG" \
  --params-file "$OVR" \
  > /tmp/lp.log 2>&1 &
LP_PID=$!

echo "=== start odom+goal publisher ==="
python3 /workspace/sentry_nav_26/scripts/diag/pub_odom_goal.py > /tmp/pub.log 2>&1 &
PUB_PID=$!

sleep "$DUR"

echo "=== capture current topics ==="
ros2 topic list > /tmp/topics.txt 2>&1 || true

echo "=== cleanup ==="
kill $PUB_PID $LP_PID $GP_PID 2>/dev/null || true
sleep 2
pkill -f sentry_local_planner_node 2>/dev/null || true
pkill -f sentry_global_planner_node 2>/dev/null || true
pkill -f pub_odom_goal.py 2>/dev/null || true

echo "===== LOCAL PLANNER LOG (tail) ====="
tail -60 /tmp/lp.log
echo "===== GLOBAL PLANNER LOG (tail) ====="
tail -15 /tmp/gp.log
echo "===== PUBLISHER LOG (tail) ====="
tail -25 /tmp/pub.log
echo "===== TOPICS ====="
cat /tmp/topics.txt 2>/dev/null
