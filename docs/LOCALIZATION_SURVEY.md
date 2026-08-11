# 全局定位方案调研（map→odom 层）

> 日期：2026-08-11
> 面向工况：**不允许提前建图 + 铁丝网边界（场外信息涌入）+ 高速自旋 + 激烈碰撞 + 赛前人群**
> 调研对象：`handshow888/fast_lio_based_localization`、`Polyacetone/HWSentryNav26`
> **结论摘要**：本仓库缺的不是"防漂移"，而是**整个 map→odom 层**。当前 `map→odom` 是静态单位阵占位。
> 真正的风险排序是：**开机对齐 > 铁丝网退化 > 赛前人群 > 累积漂移**。漂移排在最后。
> 本文所有结论标注了证据等级，未跑过的一律标 🔶/❓。

---

## 证据等级约定

| 标记 | 含义 |
|---|---|
| ✅ | 已验证：有 file:line 或可复现的命令支撑 |
| 🔶 | 推断：从代码/文档逻辑推出，**未实际运行验证** |
| ❓ | 未验证：需要实测或需要向队里/规则确认 |

---

## 优先级总览

| # | 问题 | 严重度 | 证据 | 状态 |
|---|---|---|---|---|
| 1 | `map→odom` 是静态单位阵占位，无全局定位层 | 🔴 致命 | ✅ `sim_test.launch.py:77-89` | 待做 |
| 2 | `mid360.yaml` 是纯仿真配置，真车上电即发散 | 🔴 致命 | ✅ 配置逐行标注"仿真" | 待改 |
| 3 | 仿真把铁丝网实体化，系统性高估定位性能 | 🔴 致命 | ✅ 用户确认 | 待改 |
| 4 | 碰撞时饱和帧无条件写入 iVox，污染局部地图 | 🟠 高 | ✅ `small_point_lio.cpp:122` | 待修 |
| 5 | `/global_path` 盖章 `odom`，真定位上线后 frame 错 | 🟠 高 | ✅ `global_planner_node.cpp:170` | 待改 |
| 6 | pgm 中心与 STL 中心差 ~16cm | 🟡 中 | 🔶 需读 STL bbox 确认 | 待查 |
| 7 | Point-LIO 实际漂移量未知 | 🟡 中 | ❓ `test/lio_comparison/` 为空 | 待测 |

---

## 1. 现状盘点

### 1.1 定位链路（✅ 已验证）

```
small_point_lio  ──►  odom → base_link          node.cpp:62-63     (实时)
static_transform ──►  map  → odom  全 0 单位阵   sim_test.launch.py:78-89
```

`sim_test.launch.py:77` 原注释：

```python
# === 4. 静态 map → odom TF (占位，后续由全局定位替换) ===
```

全栈活在 `odom` 系：

- `/global_path` frame_id = `"odom"`（`global_planner_node.cpp:170`）
- `sdf_map.frame_id` 默认 `"odom"`（`sdf_map.cpp:14`）
- README「当前实现状态」末行已自陈：真车适配 🟡「需替换 LIO 输入话题 + **map→odom 全局定位**」

**`map` 系目前是装饰品。**

### 1.2 坐标链验算（✅ 数值已核对，🔶 结论待确认）

```
Gazebo world 系
  ├─ STL 场地 link pose  = (14.5, 8.0, 0.2)   model.sdf
  ├─ 机器人 spawn        = ( 3.4, 9.5, 0.3)   rmuc_2025_sim.launch.py:319-324
  └─ odom 原点 ≡ spawn 点                      (LIO 开机位姿的定义)

map ≡ odom  ⇒  map 原点 = world (3.4, 9.5)

pgm: 583×324 px @ 0.05 = 29.15 × 16.2 m
     origin [-3.58, -9.44] = 图像左下角在 map 系坐标
  ⇒ 左下角 world = (3.4−3.58, 9.5−9.44) = (−0.18, 0.06)
  ⇒ 图像中心 world = (14.40, 8.16)
     vs STL link pose (14.5, 8.0)  →  差 (0.10, −0.16) m
```

**两个关键结论：**

1. ✅ **坐标锚点不依赖建图产物。** `rmuc_2025.pgm` 文件头写着 `# Created by GIMP version 2.10.30 PNM plug-in`——是照图纸手画的。整条链靠"spawn 点即原点"闭合。
   → **「不让提前建图」影响的只有漂移修正，不影响坐标对齐。**

2. 🔶 **~16cm 系统偏差待查。** 现在无害（JPS 有 clearance 余量吃得下），但**一旦拿 STL 采样 PCD 当配准 target，这 16cm 就变成系统误差**——会精确定位到一张偏了 16cm 的地图上。
   注意 `<pose>` 放的是 mesh 局部原点，不一定等于包围盒中心，需实读 STL bbox 确认。

---

## 2. 工况 → 打在哪一层

**核心拆解：五种压力打在三个不同层，只有一类真正威胁重定位层。**

| 工况 | 打在哪一层 | 现状 |
|---|---|---|
| 高速自旋（小陀螺） | LIO 层 | ✅ Point-LIO 正为此设计 |
| 碰撞 → IMU 削顶 | LIO 层 | ✅ 有饱和处理（观测层） |
| 碰撞 → 污染局部地图 | LIO 建图层 | ❌ **缺口，见 §4.1** |
| 碰撞 → 撞歪雷达 | 外参层 | ❌ 无检测 |
| 铁丝网 + 场外信息 | 重定位层 | ❌ **最难，见 §3** |
| 赛前人群 | 重定位层 | ❌ 有现成解药，见 §4.2 |
| 高速平移 | 基本无影响 | — |

### 2.1 Point-LIO 为什么扛得住自旋和碰撞（✅ 代码证据）

**IMU 是观测量，不是输入量。** 状态里带角速度和加速度（`estimator.h:26-28`，`eskf.h:24` 加速度是第 18 维），残差在 `estimator.cpp:113`：

```cpp
measurement_result.z.segment<3>(3) = linear_acceleration * imu_acceleration_scale - s.acceleration - s.ba;
```

一个坏 IMU 观测只是坏观测，被协方差降权，**不会污染状态积分**。对比 FAST-LIO 把 IMU 积分进传播模型——坏值直接固化。

**饱和是逐轴丢弃，不是丢整帧。** `estimator.cpp:118` 判 `|ω_i| ≥ satu_gyro`，`eskf.h:148/152` 跳过饱和维的更新，其余轴照常。

**逐点更新，无"帧"概念。** 小陀螺 2~3 rev/s 时一帧 100ms 覆盖 720~1080° 机体旋转，传统去畸变失效；Point-LIO 每点用自己时刻的状态投影（`estimator.cpp:50`），**畸变问题不存在**。

> **对配准算法选型的含义**：送进重定位层的 `/cloud_registered` 已是补偿过的 odom 系点云。自旋和碰撞的畸变**在上游就被吃掉了**，NDT vs GICP 之争在这个问题上是无关变量。

---

## 3. 铁丝网问题：不是外点，是**退化**

### 3.1 机制

铁丝网对 LiDAR 基本透明（丝径毫米级 vs 光斑厘米级），要么无回波要么零散噪点。视野穿过边界，涌入大量场外结构（场馆墙、看台、观众）。

**外点这一层没那么糟：** 场外结构在先验图里无对应，GICP 超过 `max_correspondence_distance` 不算 inlier，NDT 落在空体素贡献为 0。**它们大多被忽略，不是被配错。**

**真正的杀手是退化：** 有效约束点只剩三成且分布集中 → 信息矩阵在某方向接近奇异 → 该方向位姿可自由滑动而代价几乎不变。

> **关键后果：这时 fitness 看起来很漂亮**（剩下的点确实配得准），但沿退化方向的位姿完全不可信。
> **「配准收敛」≠「配准可观测」。** 单纯的 fitness 门限只挡第一种，挡不住第二种。

### 3.2 正确解法：信息矩阵特征值分析

`HWSentryNav26/odom_localizer/config/params.yaml:47-48`（✅ 已读代码）：

```yaml
min_information_eigenvalue: 1.0e+3        # 信息矩阵最小特征值下限；过小视为退化
max_information_condition_number: 1.0e+6  # 信息矩阵条件数上限；过大视为病态解
```

实现：`odom_localizer_node.cpp:751-771`，对 GICP 的 6×6 信息矩阵做特征值分解。

其 DESIGN.md §2.3 原话：

> 信息矩阵分析等价于**可观测性检查**：在长走廊等退化场景中，退化方向上几乎没有匹配约束，该方向的"修正"会被拒绝，防止定位被拉偏。

**思路不是「把场外点滤干净」（滤不干净），而是「承认某些方向不可观测，在那些方向上拒绝更新，交给 IMU/里程计兜住」。**

📚 理论出处：Zhang & Singh 2016《On Degeneracy of Optimization-based State Estimation Problems》。原文给的是 **solution remapping**（把解投影掉退化方向），比简单拒绝更精细，值得读。

---

## 4. HWSentryNav26 可借鉴项

浙大 Hello World 战队 RM2026 轮腿哨兵导航框架。里程计是 `small_glim`（GLIM 定制版，因子图固定滞后平滑），全局定位是 `odom_localizer`（GICP + 质量门控）。

### 4.1 六层质量门（✅ params.yaml:44-53）

| 门 | 值 | 挡什么 |
|---|---|---|
| `max_normalized_error` | 0.30 | 配准误差大 |
| **`min_overlap_ratio`** | **0.80** | **场外点太多** ← 直接打铁丝网问题 |
| `min_information_eigenvalue` | 1e3 | 退化 |
| `max_information_condition_number` | 1e6 | 病态 |
| `max_translation_step / max_rotation_step` | 0.2m / 0.1rad | 跳变 |
| `ema_ratio` | 0.6 | 抖动 |

对比 `fast_lio_based_localization`：**零层**（全文件 grep 不到 `hasConverged` / `getFitnessScore` / `getTransformationProbability`）。

设计哲学（DESIGN.md §2.3）：**「定位的正确性优先于频率」**——宁可 5 秒不更新，不要更新错一次。

配准源不是单帧扫描，而是 **LIO 的 iVox 局部地图快照**——体素化抑制铁丝网那种稀疏噪点回波。

### 4.2 启动区位姿按红蓝方自动选（params.yaml:20-23）

```yaml
blue: [24.1, 8.0, -0.075, 0.0, 0.0, 1.0, 0.0]   # qz=1,qw=0 → yaw 180°
red:  [3.9,  6.8,  0.302, 0.0, 0.0, 0.0, 1.0]
```

从裁判系统读颜色自动选。**本仓库 `RefereeState.msg` 加个 robot_color 字段即可抄。**

### 4.3 撞击时暂停入图 —— 本仓库真缺口

GLIM 做法（DESIGN.md §2.2）：IMU 饱和帧丢弃不入图，此后**暂停插入 0.5s**。理由：饱和帧位姿不可信，插进局部地图会**污染后续所有匹配目标**。

本仓库 `small_point_lio.cpp:122`（✅ 已验证）：

```cpp
estimator.ivox->add_point(estimator.point_odom_frame);   // 无条件插入
```

紧跟 `update_point()` 之后，**无任何饱和门控**。碰撞瞬间 IMU 削顶 → 状态不可信 → 投影出的点位置错 → **永久写进 iVox**。

Point-LIO 在观测层已丢弃饱和轴（`estimator.cpp:118`），但**建图层无对应保护**。
修法：记录最近饱和时刻，`:122` 前加时间窗判断。

### 4.4 但它建图了

`params.yaml:26`：`cloud_filename: "RMUC202608af.pcd"`，且是重度建图——离线两阶段因子图 + 子图内 VGICP + 全局回环 + 3D-DDA 去动态物体（README 第 6 点）。

**它没有回答「不让建图」这个约束。** 但它证明了：合成先验图路线要能用，**必须**配这套门控。

---

## 5. 赛前人群

**为什么比一般动态障碍严重一个量级：**

- 单帧点云里人体可能占 30~50%，不是「少量外点」
- 人在移动，每次配准被拽的方向不同 → 结果不收敛到一致值
- **恰好是初始化时刻**——初始重定位错了，整局全局路径全错

NDT/GICP 的鲁棒性都是软鲁棒性，设计上只吃个位数百分比外点。**换配准算法解决不了。**

**解药（已有现成数据链）：** `sentry_msgs/msg/RefereeState.msg` 里就有

```
uint8 game_progress   # 0未开始 1准备 2自检 3五秒倒计时 4比赛中 5结算
```

`game_progress` 低于阈值时**完全冻结 map→odom**，用固定出生点先验；人清场后再启用配准。
❓ 卡 `>= 2`（自检）还是 `>= 3`（倒计时）需跟队里确认赛制流程。

---

## 6. 技术备忘

### 6.1 NDT vs ICP vs GICP —— 为什么重定位场景选 NDT

**本质区别：** ICP/GICP 每次迭代都要重新问「谁对应谁」；NDT 把 target 预编译成**概率势场**，只需问「我落在场的哪一格」。

对应关系是位姿的**离散函数**——位姿动一点，最近邻可能换人，代价面出现折痕。NDT 是高斯之和，**处处可微**，可直接上 Newton + More-Thuente line search。

| 优势 | 说明 |
|---|---|
| target 固定 ⇒ 预处理全摊销 | GICP 的 target 协方差也能预算，**但对应关系不能**（依赖当前位姿）。NDT 连这步都没有 |
| 收敛盆地大一个数量级 | ICP 吸引域 ≈ 点间距（cm 级）；NDT ≈ 体素分辨率（1m） |
| 密度不匹配无系统偏差 | 先验图几十万点 vs 单帧 2 万点，ICP 最近邻会被往稠密侧拽 |
| 对「地图比现实少东西」免疫 | 多余源点落在空体素，贡献 0——**这条让合成地图路线可行** |

**劣势（诚实说）：** 精度上限低于 GICP（体素内多朝向面时单高斯退化成胖椭球，墙角/坡道边缘尤甚）；多一个 resolution 超参；体素边界不连续。

**帧间配准场景优势归零**——两边都在变，所以 LOAM 系 / FAST-LIO / Point-LIO **内部都不用 NDT**。

### 6.2 时间预算（🔶 复杂度估算，未实测）

| 阶段 | 量级 | 何时 | 并行 |
|---|---|---|---|
| 加载 PCD | 0.1~5 s | 开机 | 取决于 binary/ASCII |
| `setInputTarget` 建体素高斯 | ~100 ms | 开机 | ❌ **单线程** |
| 单次 `align` | 10~30 ms | 每次配准 | ✅ 4 线程 |
| 初始收敛（好初值） | 1 次 ≈ 20 ms | 解冻瞬间 | — |

✅ 已验证：`ndt_omp_impl.hpp:206` 只对 **源点** 并行（`#pragma omp parallel for`）；`voxel_grid_covariance_omp_impl.hpp` 的 `applyFilter` **无任何 omp pragma**。
→ **`num_threads` 只加速 align，不加速建图；但建图只做一次。**
→ align 耗时与**地图大小几乎无关**，只与源点数、迭代次数成正比。

**五秒倒计时窗口有 ~200 倍余量。真正的约束是人为的 1Hz 限流，别照抄。**
正确设计：解冻瞬间连打 8~10 次不限流（~200ms）取一致性最好的；稳态再降到 1~2Hz 省 CPU。

压 align 耗时的杠杆（按顺序）：`voxel_leaf_size` 0.1→0.2 → `max_scan_range` 100→20 → `maximum_iterations` 20→10 → **别动 `resolution`**（它管收敛盆地）。

### 6.3 `fast_lio_based_localization` 评估结论

上游是 `BruceXSK/fast_lio_localization` 的 ROS2 参数化 fork，11 次提交，2024-04-30 后无更新。**架构选型对，工程防护为零。**

不可直接 drop-in 的原因：

1. 订阅 `/cloud_registered_body`（body 系），本仓库只发 `/cloud_registered`（`node.cpp:146`，odom 系）
2. 需要 3D PCD，本仓库只有 pgm + STL
3. 零质量门控

**若自己写，反而更简单：** 点云已在 odom 系 ⇒ NDT 把它对到 map 系先验图，`getFinalTransformation()` **直接就是 `T_map←odom`**。不需要订阅 `/Odometry`，message_filters 和那个脆弱的 exact TimeSynchronizer 整个删掉。

⚠️ 移植必踩的坑：`min_scan_range/max_scan_range` 用 `hypot(p.x,p.y)` 按 body 系原点算距离，世界系点云照抄会变成「按到地图原点的距离过滤」，**静默错**。

---

## 7. 候选路线

| 路线 | 说明 | 规则风险 | 备注 |
|---|---|---|---|
| **A. STL 采样 PCD** | 从 `rmuc_2025.stl`（5.5MB, ~11 万三角面）表面均匀采样 | ❓ 性质同 pgm（都从图纸生成），需确认 | 首选，但先修 §1.2 的 16cm |
| **B. pgm 2D 配准** | 扫描投影成 2D 轮廓，与 pgm 相关性匹配 | ✅ 零争议（不新增素材） | 只修 x/y/yaw，保底方案 |
| **C. 纯出生点锚定** | 不做修正，靠精确标定出发区位姿 | ✅ 零争议 | 取决于 §7.1 的漂移量 |

### 7.1 但第一件事是量漂移

**如果 Point-LIO 7 分钟漂移 < 20cm，整个重定位层就是过度设计。** 三条路线选哪条都得先有这个数。

`test/lio_comparison/` 目前为空。而测量所需的一切已就位——`rmuc_2025_sim.launch.py:433-437` 已桥出 Gazebo 真值：

```python
"/model/mecanum_bot/pose@geometry_msgs/msg/PoseStamped@gz.msgs.Pose"
```

```bash
ros2 bag record /Odometry /model/mecanum_bot/pose
```

⚠️ **caveat**：仿真 IMU/点云噪声理想化，无真实碰撞、无真实小陀螺动力学。**测出的是漂移下界，真车只会更差。** 但能给出正确数量级，且可在仿真里控制变量（主动加小陀螺指令）。

### 7.2 合成图路线的关键警告

⚠️ **`min_overlap_ratio` 必须重新标定。** HWSentryNav26 的 0.80 是针对真实建图产物定的。合成图缺场外结构、缺细节、缺地面起伏，overlap 天然低——**照抄 0.80 会导致永远配准失败，退化成纯里程计还不自知。**

尺寸公差风险（图纸 vs 实际拼装，厘米级）**比直觉小**：`resolution: 1.0` 完全吃得下。

---

## 8. 真车配置检查表（`config/mid360.yaml` 目前是纯仿真值）

**比选 NDT 还是 GICP 重要一个量级。**

| # | 参数 | 现值 | 真车应为 | 后果 |
|---|---|---|---|---|
| 1 | `acc_norm` | 9.81 | **1.0** | Livox 真机 IMU 输出单位是 **g**，不改则加速度差 9.81 倍，ESKF 当场发散 |
| 2 | `lidar_type` | `standard_pointcloud2` | `livox_custom_msg` | 少了**逐点时间戳**——Point-LIO 逐点更新的前提，小陀螺下直接暴毙 |
| 3 | `extrinsic_T` | `[0,0,0]` | 查 Livox 手册 | 这是 **LiDAR→IMU** 外参（≠ `run_sim.launch.py` 里那个 base_link→lidar 的 `[0.2,0,0.125]`，两者现在都对） |
| 4 | `satu_acc` | 30.0 | = 硬件量程 | 注释自陈「调高仿真饱和阈值」。设低了正常激烈运动数据大量被丢，退化成纯 LiDAR 里程计 |
| 5 | `satu_gyro` | 35.0 | ❓ 核实手册 | = 2005 °/s；若 Mid360 陀螺量程 ±2000 dps 则设得很准（正好卡量程）。小陀螺 3 rev/s ≈ 18.8 rad/s 不会误触发 |

> 另：submodule 自带的 `small_point_lio.launch.py:43` 静态 TF 用 `livox_frame`，而 config 里 `lidar_frame: livox_lidar`——**对不上**，`node.cpp:66` 的 lookupTransform 会失败，odom 根本不发布。仿真走 `run_sim.launch.py`（用 `livox_lidar`，一致）所以未暴露。**真车别用错那个 launch。**

---

## 9. 行动清单

### 立刻（本周）

1. **让仿真诚实起来** —— `rmuc_2025_world.world` 去掉边界 collision（保留 visual），场外 5~15m 摆几个方块当场馆结构。
   > 现在仿真把铁丝网实体化了，**系统性高估定位性能，且高估的正是最危险的那个维度**。当前仿真测出的任何定位指标都不能外推真车。
2. **量漂移** —— 录 `/Odometry` + `/model/mecanum_bot/pose`，出 ATE/RPE 曲线，小陀螺开/关对比。一次实验同时回答「漂移多大」和「退化多严重」。
3. **确认 STL bbox** —— 定 §1.2 那 16cm 是真偏差还是 mesh 原点定义差异。

### 短期

4. 补 `map→odom` 层：出生点锚定 + `game_progress` 门控 + 退化检测（信息矩阵特征值）+ 两段式节奏
5. `small_point_lio.cpp:122` 加饱和时间窗门控（§4.3）
6. `RefereeState.msg` 加 robot_color 字段

### 中期

7. `/global_path` 迁到 `map` 系（或显式变换）——**真定位上线后这是必改项，别低估工作量**
8. 真车配置逐项过 §8 检查表
9. 碰撞后外参自检（静止时跑一次配准，fitness 劣化即报警）

---

## 10. 未验证 / 待澄清

| # | 项 | 类型 | 备注 |
|---|---|---|---|
| 1 | 「不让提前建图」规则原文禁止范围 | ❓ 需确认 | 决定路线 A 是否可行；pgm 手画都允许，从 STL 生成性质相同 |
| 2 | `game_progress` 卡 2 还是 3 | ❓ 需确认赛制 | 影响清场判据 |
| 3 | Point-LIO 实际漂移量 | ❓ 待实测 | 决定重定位层是否必要 |
| 4 | §6.2 全部时间数字 | 🔶 复杂度估算 | 有 PCD 后开 `debug: true` 实测；注意它只测 align 不含 `setInputTarget`，后者更值得单独计时 |
| 5 | STL 中心 vs pgm 中心 16cm | 🔶 推断 | 需实读 STL bbox |
| 6 | Mid360 陀螺/加速度计实际量程 | ❓ 查手册 | 决定 `satu_gyro`/`satu_acc` |
| 7 | 合成图的 `min_overlap_ratio` 阈值 | ❓ 待标定 | 照抄 0.80 会永远失败 |
| 8 | `sentry_serial` 的 `game_progress` 是否已实际发布 | ❓ 待确认 | 门控方案的前提 |

---

## 参考

- `handshow888/fast_lio_based_localization` —— FAST-LIO + NDT 两层重定位，架构对、防护零
- `Polyacetone/HWSentryNav26` —— 浙大 HelloWorld 战队 RM2026 轮腿哨兵，GLIM + GICP + 六层门控
- `koide3/glim` —— 因子图 LiDAR-IMU SLAM（small_glim 的上游）
- Zhang & Singh 2016, *On Degeneracy of Optimization-based State Estimation Problems* —— 退化检测与 solution remapping
- Magnusson 2009, *The Three-Dimensional Normal-Distributions Transform* —— 3D-NDT 原始论文
- Segal et al. 2009, *Generalized-ICP* —— plane-to-plane GICP
