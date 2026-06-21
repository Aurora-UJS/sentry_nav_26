# sentry_nav_26 代码审计与改进建议

> 面向工况：**无先验地图 + 极限钻洞 + 高速移动**（RMUC 哨兵）
> 审计范围：local planner、path_searching、plan_env、LIO 仿真接入
> 结论摘要：架构选型正确、代码干净、自我认知诚实（README 完成度表到位）。
> 当前短板恰好集中在高速 + 钻洞 + 无先验三个方向，且多数已在 README 标记 ❌/🟡。

---

## 优先级总览

| # | 问题 | 严重度 | 类别 | 状态 |
|---|---|---|---|---|
| 1 | 单线程 executor，重规划阻塞 50Hz 控制 | 🔴 致命 | 工程 | 待修 |
| 2 | 点云分支无 raycast 清除，动态障碍永久残影 | 🔴 致命 | 工程/正确性 | README 已标 |
| 3 | 重规划仅 2Hz，高速下反应不足；文档写 10Hz 自相矛盾 | 🟠 高 | 工程 | 待修（依赖 #1）|
| 4 | footprint = 半径 0.3 五点十字，无朝向，对角线漏检 | 🟠 高 | 算法/钻洞 | 待修 |
| 5 | 单层 2D 占用，狗洞/净空判定缺失 | 🟠 高 | 算法/钻洞 | README 已标 ❌ |
| 6 | "MPC" 实为 PD+前馈，且每 tick 重建常量矩阵 | 🟡 中 | 算法 | 待修 |
| 7 | MINCO dist0=0.05 固定，无速度相关安全裕度 | 🟡 中 | 算法/高速 | 待修 |
| L1 | 仿真 IMU 缺重力 → 上下坡 Z 漂移（点云上天入地）| 🔴 致命 | LIO/仿真 | 待验证+修 |
| L2 | standard 点云逐点时间戳全相同 → 去畸变失�� | 🟠 高 | LIO | 待修 |

---

## 工程层

### #1 单线程 executor 阻塞控制回路（头号问题）

`sentry_local_planner_node.cpp:376` 使用 `rclcpp::spin(node)`，单线程。
`controlLoop`（50Hz）、`replanCallback`（2Hz）、`odomCallback` 全部串行在同一线程。

- A\* + MINCO 单次约 8~20ms。重规划一触发，50Hz 控制回路即丢拍，`/cmd_vel` 抖动甚至断流。
- 3 m/s 下，丢一拍 = 走 6cm 无控制更新，贴墙钻洞场景直接撞。

**修复方向**
- 改 `MultiThreadedExecutor`。
- 规划放入独立 `CallbackGroup`（Reentrant 或单独 MutuallyExclusive），与控制/odom 隔离。
- 轨迹用 `std::atomic<std::shared_ptr<const Traj>>` 双缓冲：规划线程构建新轨迹后原子 swap，控制线程只读当前指针。

> 注意：README 提到的 "trajectory swap 原子化（修 race）" 在当前单线程下是**伪 race**（不会真并发）。改多线程后才变成真 race —— 两者必须一起改。

### #2 点云分支无 raycast 清除（无先验动态场致命）

`map_core.cpp:242` 的 `raycast()` 已实现 log-odds miss 衰减，但 cloud 输入分支未调用（README 已承认）。
log-odds 只增不减 → 场上其他机器人移动后，在地图里留下永久"鬼墙"，越积越多最终堵死通道。

**修复方向**
- 在点云融合时，对每个命中点沿"传感器原点 → 命中点"做 raycast，路径上的体素施加 `logodds_miss`。
- 注意 ring-buffer 地址换算（`toAddress` 已处理 ring_offset），传感器原点用当前 odom 位置。
- 中期可升级到 STVL 风格时间衰减 + LiDAR 视锥清除（README roadmap 已列）。

### #3 重规划频率

`planner_params.yaml:96` `replan.frequency: 2.0`，但 README 数据流图写 10Hz —— 文档与代码不一致，先统一。
3 m/s 下两次重规划间车走 1.5m，局部图仅 8m，动态障碍来不及反应。
**提到 ≥10Hz，但必须先解决 #1**，否则只会让控制更卡。

---

## 算法层（直击钻洞 / 高速）

### #4 footprint 模型过弱

`kinodynamic_astar.cpp:54-58`：footprint 只采 中心 + 上/下/左/右 4 点，膨胀 0.05（纯去噪）。

- **对角线漏检**：缝略窄于 2r、障碍从 45° 方向伸入时，四个正方向采样点之间漏检 → 贴墙切角。
- **无朝向**：搜索状态 `(x,y,vx,vy)` 不含 yaw。麦轮全向可化解部分，但车体非圆，极限缝内是命门。

**修复方向（性价比最高的一刀）**
- 改为**轮廓边界密集采点**（M=12~20，仅采边界不填内部）。
- 加**两级半径剪枝**：
  1. 查中心点 ESDF 距离 `d`；
  2. `d ≥ 外接圆半径` → 整车安全，跳过 M 点；
  3. `d < 内切圆半径` → 整车碰撞，直接判死；
  4. 仅 `内切 ≤ d < 外接` 模糊带才查 M 个边界点。
- 平均代价接近膨胀法，却堵住切角。采样间距须 < 最薄障碍宽度。

### #5 单层 2D 占用，无狗洞/净空

`planner_params.yaml:14` `cloud_max_height: 0.8` 把 0.8m 以下所有点拍扁成一张图：
- 可钻的低矮通道 → 被当实墙；
- 高处横杆 → 被当地面。

**修复方向（中期）**
- 多层占用图：ground / body / head 分层，body 层判断车体高度内是否有障碍，head/ground 联合判净空 → 实现狗洞通行。

### #6 "MPC" 名不副实

`mpc_controller.hpp`：
- 每 tick 重建 `S/T/Q_bar/R_bar`，但这些只依赖 config、为常量，应在 `buildModel()` 算一次缓存。
- `trajectory_tracker.cpp:49` 实际下发 `vel_ref + kp*(pos_ref-pos) + acc_cmd*lookahead`，MPC 解出的最优加速度仅作 ×0.2 前馈微调 —— 主体是 P+前馈，付了 LDLT 的钱跑的是 PD。

**修复方向**
- 缓存常量矩阵。
- 要么老实做带不等式硬约束的 QP-MPC（OSQP，roadmap 已列），把速度/加速度/可行域约束放进去；要么坦诚命名为 PD+前馈。

### #7 MINCO 安全裕度与速度无关

`planner_params.yaml:104` `dist0: 0.05` 固定常量。高速需要**随速度增长的刹车裕度**（v 越大离墙越远）。
当前 5cm 常量不足以吸收 LIO 抖动 + 控制延迟。

**修复方向**
- `dist0_eff = dist0_base + k_v * |v|`，或在碰撞代价里加入速度相关项。

---

## LIO / 仿真层

### L1 仿真 IMU 缺重力 → 上下坡 Z 漂移（点云上天入地）

**现象**：平地正常，一上坡/下坡/下台阶，odom 在 Z 方向漂移，点云上天入地。

**根因**
- `mid360.yaml` 开启 `fix_gravity_direction: true` + `acc_norm: 9.81`：LIO 假设静止时 IMU 加速度计读数 ≈ `[0,0,+9.81]`（真实 IMU 测比力）。
- 但 gz-sim IMU 插件默认输出**纯运动加速度，静止时为 `[0,0,0]`，不含重力**。`model.sdf:518-529` 的 IMU 只配了高斯噪声，无任何重力补偿设置。
- 结果：LIO 重力基准错误，Z 加速度积分基准全乱。平地匀速 Z 加速度≈0 被噪声掩盖；上下坡出现真实 Z 加速度时，无正确重力基准解释 → 积分漂移。

**已有的治标痕迹**（yaml 注释）：`imu_meas_acc_cov 0.01→0.1`、`acceleration_cov 500→100` 都是压低对 IMU 加速度的信任硬压 Z 抖动 —— 平地能糊弄，坡道上真实 Z 加速度要么被当噪声丢（迟钝/滑步），要么积分发散（上天）。

**验证步骤（先做这个再动手）**
```bash
# 车静止，看 z 分量
ros2 topic echo /livox/imu --field linear_acceleration
```
- 读到 `z ≈ 0` → 实锤缺重力，走修复 1 或 2。
- 读到 `z ≈ ±9.81` → 重力在，检查符号/坐标系与 `gravity: [0,0,-9.810]` 约定是否一致。

**修复方向**
1. **改 SDF 让 IMU 输出含重力**（首选）：gz-sim IMU 配重力补偿/方向，使静止读数为 `[0,0,+9.81]`，满足 `fix_gravity_direction` 前提。
2. **SDF 改不动时在 LIO 侧适配**：关闭 `fix_gravity_direction`，或在 IMU 预处理给 z 补 `+9.81`（small_point_lio 当前未必有现成开关，可能需改预处理）。

### L2 standard 点云逐点时间戳全相同 → 去畸变失效

`standard_pointcloud2.h:37`：一帧内**所有点 timestamp 被赋成同一个 `msg.header.stamp`**。
Point-LIO 靠逐点精确时间做运动去畸变。全帧同一时间 = 告诉 LIO 这 100ms 的点是同一瞬间打的。
高速 + 转向 + 上下坡时一帧内姿态变化大 → 点云拖影 → 喂坏平面匹配，与 L1 叠加放大 Z 飘。

> 对比：Livox 自定义消息分支 `livox_custom_msg.h:33` 用 `offset_time` 正确；仅仿真用的 standard PointCloud2 分支坏（RGL 输出的标准点云不带 per-point 时间字段）。

**修复方向**
- 临时：按扫描顺序在 `[0, 1/update_rate)` 内线性插值近似：
  `timestamp = msg_timestamp + (i / size) * scan_period`
- 彻底：让 RGL 插件输出带 `t`/`timestamp` 字段的点云，适配器直接读。
- 注意 small_point_lio 是 submodule，改动需在子模块内提交。

---

## 做得好的地方（非客套）

- kinodynamic A\*（含 OBVP shot）给动力学可行种子轨迹，对高速是正确选择，优于纯几何 A\*。
- ESDF 为正经 ring-buffer + Felzenszwalb 两遍可分离 DT，滑窗换原点处理得当。
- MINCO + L-BFGS 带 20ms 时间预算，有实时意识。
- yaw_rate/yaw_acc 限幅是为防止 LIO 点云配准抖发散 —— 说明在实车上踩过坑。
- README 自带完成度表 + 已知问题，工程诚实度高。

---

## 建议修复顺序

1. **L1 仿真 IMU 重力**（先 echo 验证）—— 不解决，仿真里一切坡道测试都不可信。
2. **#1 多线程 + 轨迹双缓冲** —— 高速表现的总闸。
3. **#2 点云 raycast 清除** —— 无先验动态场的地图正确性。
4. **#3 重规划提到 10Hz**（依赖 #1）。
5. **L2 standard 点云逐点时间戳** —— 改善高速/坡道拖影。
6. **#4 footprint 边界采点 + 两级剪枝** —— 极限钻洞性价比最高的一刀。
7. 中期：**#5 多层占用（狗洞）** + **#6 QP-MPC** + **#7 速度相关裕度**。
