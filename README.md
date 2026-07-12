# Sentry Nav 26

面向 RoboMaster 哨兵机器人的模块化自主导航栈，基于 **ROS 2 Jazzy** + **Gazebo Harmonic**，提供从 LIO 定位、在线建图、全局规划到局部轨迹生成与跟踪的完整闭环。

---

## 特性

- **LIO 定位**：基于 [Small Point-LIO](https://github.com/Aurora-UJS/small_point_lio)，输出高频 `/Odometry` 与配准点云 `/cloud_registered`
- **在线 2D 建图**：log-odds 占用栅格 + 高程图 + Felzenszwalb 两遍 ESDF（`plan_env`）
- **全局规划**：JPS（Jump Point Search）+ 静态先验 PGM 地图（`sentry_global_planner`）
- **局部规划**：Kinodynamic A* + 多项式 Shot 直达（`path_searching`）
- **轨迹优化**：MINCO 五次多项式 + L-BFGS（NLopt），平滑/避障/可行性多目标
- **轨迹跟踪**：50 Hz LDLT-MPC（10 步预测 horizon），独立 yaw 控制器（窄道对齐 / 开阔旋转）
- **仿真环境**：RMUC 2025 赛场（[`rm_sim_26`](https://github.com/Neomelt/rm_sim_26)）+ Livox 雷达 / IMU 桥接
- **中间件**：推荐 `rmw_zenoh_cpp`，大点云吞吐与多机场景表现优于 DDS

---

## 仓库结构

```
sentry_nav_26/
├── src/
│   ├── sentry_bringup/          # 顶层 launch + RViz 配置 + 静态地图 (rmuc_2025)
│   ├── sentry_sim/              # ros_gz_bridge 桥接配置
│   ├── rm_sim_26/               # [submodule] RMUC 2025 Gazebo 仿真世界
│   ├── small_point_lio/         # [submodule] LiDAR-Inertial Odometry
│   ├── sensor_driver/
│   │   └── livox_ros_driver2/   # [submodule, 真车需] Livox 驱动
│   │
│   ├── plan_env/                # 在线感知层：占用 + 高程 + ESDF
│   │   ├── map_core.cpp/.hpp        # 环形缓冲、滑动窗口、ESDF 计算
│   │   ├── sensor_processor.cpp     # 点云/激光 → occupancy + elevation
│   │   ├── sdf_map.cpp              # ROS 接口节点 (订阅/参数/发布)
│   │   ├── edt_environment.cpp      # 给规划器用的距离场查询接口
│   │   └── obj_predictor.cpp        # 动态物体预测占位
│   │
│   ├── path_searching/          # Kinodynamic A* + 多项式求解器
│   │   ├── kinodynamic_astar.cpp    # 二阶动力学 A* + OBVP shot
│   │   └── polynomial_solver.hpp    # 三/四次方程解析求根
│   │
│   ├── sentry_global_planner/   # JPS 全局路径搜索
│   │   ├── global_planner_node.cpp
│   │   ├── jps_searcher.hpp         # JPS 算法实现
│   │   └── global_map.hpp           # PGM 加载 + 静态 ESDF + OnlineMapProxy
│   │
│   ├── sentry_local_planner/    # MINCO + MPC 局部规划与控制
│   │   ├── sentry_local_planner_node.cpp   # 顶层节点（10Hz replan + 50Hz control）
│   │   ├── minco_trajectory.cpp            # MINCO 轨迹生成 + L-BFGS 优化
│   │   ├── trajectory_tracker.cpp          # MPC ↔ 轨迹接口
│   │   └── mpc_controller.hpp              # LDLT 求解的二阶 MPC
│   │
│   └── sentry_msgs/             # 自定义 msg/srv/action 模板
│
├── test/                        # launch 集成测试占位
├── debug_planner.sh             # GDB 调试脚本
├── cyclonedds.xml               # 备用 CycloneDDS 配置（默认推荐 Zenoh）
└── README.md
```

---

## 系统数据流

```
LiDAR + IMU
   │
   ▼
small_point_lio  ──►  /Odometry (高频位姿)
                  └►  /cloud_registered (世界系点云)
   │
   ├──► plan_env::SDFMap
   │       ├─ 点云 / 激光 → log-odds occupancy
   │       ├─ 滑窗 ESDF (Felzenszwalb 两遍可分离 EDT)
   │       └─ 高程图 + 邻域坡度
   │
   ├──► sentry_global_planner (JPS, 静态先验地图)
   │       └─ /global_path  (nav_msgs/Path)
   │
   └──► sentry_local_planner
           ├─ 订阅 /Odometry, /goal_pose, /global_path
           ├─ KinodynamicAstar 在 EDTEnvironment 上搜索 (10Hz)
           ├─ MINCO 五次轨迹 + L-BFGS 优化 (粗 8ms / 精 12ms)
           └─ MPC 跟踪 (50Hz) ──►  /cmd_vel
```

---

## 关键话题与接口

### 输入
| 话题 | 类型 | 来源 |
|---|---|---|
| `/Odometry` | `nav_msgs/Odometry` | small_point_lio |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | small_point_lio |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz 2D Goal Pose |

### 中间
| 话题 | 类型 | 发布者 | 订阅者 |
|---|---|---|---|
| `/global_path` | `nav_msgs/Path` | sentry_global_planner | sentry_local_planner |
| `/sdf_map/occupancy` | `sensor_msgs/PointCloud2` | plan_env | RViz 调试 |
| `/sdf_map/esdf` | `sensor_msgs/PointCloud2` | plan_env | RViz 调试 |

### 输出
| 话题 | 类型 | 用途 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 底盘速度命令 |
| `/planning/trajectory` | `nav_msgs/Path` | 当前 MINCO 轨迹（可视化）|
| `/planning/trajectory_markers` | `visualization_msgs/MarkerArray` | 调试可视化 |

---

## 环境依赖

- **OS**：Ubuntu 24.04（Noble）
- **ROS 2**：Jazzy Jalisco
- **仿真**：Gazebo Harmonic
- **C++**：≥ C++17
- **第三方库**：Eigen3、PCL、OpenCV、NLopt、tf2

---

## 安装

### 1. 系统依赖

```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-gz-tools-vendor ros-jazzy-gz-sim-vendor ros-jazzy-ros-gz \
    ros-jazzy-tf2-sensor-msgs ros-jazzy-tf2-eigen \
    ros-jazzy-rmw-zenoh-cpp \
    ros-jazzy-teleop-twist-keyboard \
    libnlopt-dev libnlopt-cxx-dev \
    libpcl-dev libeigen3-dev libopencv-dev
```

### 2. 克隆仓库（含 submodule）

```bash
cd ~
git clone --recurse-submodules <your-repo-url> sentry_nav_26
cd sentry_nav_26

# 若已 clone 但未拉 submodule
git submodule update --init --recursive
```

### 3. 编译

```bash
source /opt/ros/jazzy/setup.bash
cd ~/sentry_nav_26
colcon build --symlink-install
```

### 4. 配置中间件（推荐 Zenoh）

```bash
# 写进 ~/.bashrc 或 ~/.zshrc
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

每次启动栈前先单独跑 zenoh router（仅在第一次或重启路由时需要）：

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

> Zenoh 在大点云（LiDAR / 配准点云）传输和多机场景下吞吐与延迟均优于 DDS，推荐作为本项目默认 RMW。如要回退到 CycloneDDS，可使用仓库自带的 `cyclonedds.xml`。

---

## Fedora / Podman 容器开发

Fedora 主机可用 Podman 运行 Ubuntu 24.04 + ROS 2 Jazzy 开发环境，避免在 Fedora 上直接适配 ROS 生态。容器镜像安装当前工程编译与仿真需要的 ROS/Gazebo 依赖；代码目录会挂载进容器，`build/`、`install/`、`log/` 仍生成在当前仓库。

### 1. 主机准备

```bash
sudo dnf install -y podman xorg-x11-server-utils

# Wayland 桌面通常也保留 XWayland；允许本机用户连接 X11，供 RViz/Gazebo GUI 使用
xhost +SI:localuser:$(id -un)
```

若仓库 submodule 尚未拉取，可直接用容器执行：

```bash
./scripts/podman-dev.sh submodules
```

该命令会拉取当前仿真启动必需的 `src/rm_sim_26` 与 `src/small_point_lio`。

### 2. 构建开发镜像

```bash
./scripts/podman-dev.sh build-image
```

镜像默认标签为 `localhost/sentry-nav-26:jazzy`，可通过 `SENTRY_NAV_IMAGE` 覆盖。
构建时默认把 Ubuntu apt 源切到清华镜像，可通过 `SENTRY_NAV_UBUNTU_MIRROR` 覆盖；如 ROS 源访问不稳定，也可设置 `SENTRY_NAV_ROS_APT_MIRROR`。

### 3. 安装依赖并编译

```bash
# 可选：让 rosdep 补齐 submodule 或未来新增包的系统依赖
./scripts/podman-dev.sh deps

./scripts/podman-dev.sh build
```

也可以进入容器手动执行任意 ROS 命令：

```bash
./scripts/podman-dev.sh shell
source install/setup.bash
colcon build --symlink-install
```

### 4. 容器内运行仿真导航

推荐开三个终端，分别运行：

```bash
# 终端 1: Zenoh router
./scripts/podman-dev.sh zenoh

# 终端 2: 仿真 + 导航栈 + RViz/Gazebo
./scripts/podman-dev.sh bringup

# 终端 3: 键盘遥控
./scripts/podman-dev.sh teleop
```

`bringup` 会以 `start_teleop:=false` 启动，避免容器内强依赖 `gnome-terminal`；需要遥控时用第三个终端单独运行 `teleop`。

如需执行自定义命令：

```bash
./scripts/podman-dev.sh exec ros2 pkg list
./scripts/podman-dev.sh exec ros2 launch sentry_local_planner planner.launch.py
```

> 图形界面依赖主机 X11/XWayland 与 `/dev/dri` 显卡设备透传。若 RViz/Gazebo 无法显示，先确认 `echo $DISPLAY` 有值，并重新运行 `xhost +SI:localuser:$(id -un)`。

---

## 快速上手

### 一键启动（仿真）

```bash
# 终端 1: 启动 zenoh router
ros2 run rmw_zenoh_cpp rmw_zenohd

# 终端 2: 启动仿真 + 导航栈
source ~/sentry_nav_26/install/setup.bash
ros2 launch sentry_bringup bringup.launch.py
```

`bringup.launch.py` 通过 `sim_test.launch.py` 串起：

| 时序 | 组件 |
|---|---|
| t = 0 | RMUC 2025 Gazebo 世界 + ros_gz_bridge |
| t = 0 | RViz2 + 静态 `map → odom` TF（占位）|
| t = 0 | 新终端启动 teleop_twist_keyboard |
| t = 6s | small_point_lio（等待 Gazebo 就绪）|
| t = 12s | sentry_local_planner + sentry_global_planner |

延迟启动是为了让 LIO 在规划器启动前完成初始化。

### 发送目标点

启动后在 RViz2 中点击 **2D Goal Pose**，目标会发布到 `/goal_pose`，全局规划器输出 `/global_path`，局部规划器跟踪并发出 `/cmd_vel`。

### 单独调试某节点

```bash
# 只启动局部规划器（需要外部 odom + goal）
ros2 launch sentry_local_planner planner.launch.py

# 只启动全局规划器
ros2 launch sentry_global_planner global_planner.launch.py \
    map_yaml:=$HOME/sentry_nav_26/src/sentry_bringup/map/rmuc_2025.yaml

# GDB 启动局部规划器（崩溃排查）
./debug_planner.sh
```

### 重启 LIO（独立调试）

```bash
source install/setup.zsh
timeout 15 ros2 run small_point_lio small_point_lio_node \
    --ros-args --params-file install/small_point_lio/share/small_point_lio/config/simulation.yaml
```

---

## 关键参数

### 局部规划器（`sentry_local_planner/config/planner_params.yaml`）

| 项 | 默认 | 说明 |
|---|---|---|
| `controller.control_freq` | 50 Hz | MPC 控制频率 |
| `replan.frequency` | 10 Hz | 轨迹重规划频率 |
| `mpc.horizon` | 10 | 预测步数（horizon = N · dt = 0.2s）|
| `mpc.q_pos / q_vel / r_acc` | 10 / 1 / 0.1 | MPC 状态/输入权重 |
| `minco_opt.max_time_s` | 0.02 | L-BFGS 总预算（粗 40% / 精 60%）|
| `search.max_vel / max_acc` | — | 动力学搜索的速度/加速度上限 |

### 全局规划器（`sentry_global_planner/config/global_planner_params.yaml`）

| 项 | 默认 | 说明 |
|---|---|---|
| `global_map.mode` | `prior` | `prior` = 加载 PGM；`online` = 用 plan_env 实时 ESDF（**未完成**） |
| `jps.safety_dist` | 0.3 m | ESDF 小于此值视为不可通行 |
| `jps.esdf_weight` | 2.0 | ESDF 代价权重，越大越远离障碍 |
| `jps.waypoint_spacing` | 1.0 m | 简化后路径点最小间距 |

### 在线建图（`plan_env`，由 launch 注入）

| 项 | 默认 | 说明 |
|---|---|---|
| `sdf_map.resolusion_` | 0.05 m | 体素分辨率（**注意 yaml 里键名拼写**）|
| `sdf_map.local_update_range_x/y` | 3.0 m | 局部窗口半径 |
| `sdf_map.obstacles_inflation` | 视 yaml | 膨胀半径 |
| `sdf_map.max_slope_deg` | 17° | 高程坡度阈值 |
| `sdf_map.logodds_*` | hit 0.85 / miss -0.4 | 贝叶斯更新参数 |

---

## 当前实现状态

| 模块 | 完成度 | 备注 |
|---|---|---|
| LIO 定位 | ✅ | 通过 submodule |
| 在线 2D occupancy + ESDF | ✅ | log-odds + Felzenszwalb |
| 高程图 / 坡度估计 | 🟡 | `elevation_buffer_` 已建，坡度只用于点云过滤，未进入代价 |
| 多层占用（狗洞净空检测）| ❌ | 未实现 |
| 时间衰减体素 (STVL) | ❌ | log-odds 永久累积，无衰减 |
| 视锥清除 | ❌ | 未实现 |
| 可通行性融合层 | ❌ | 未实现 |
| 全局规划 (JPS, 静态先验) | ✅ | 完整可用 |
| 全局规划在线模式 | ❌ | `OnlineMapProxy` 已搭好，未接通 |
| Kinodynamic A* | ✅ | 二阶动力学 + OBVP shot |
| MINCO 轨迹优化 | ✅ | 五次多项式 + L-BFGS |
| MPC 跟踪 | ✅ | LDLT，软约束 |
| 执行中轨迹安全监控 | ✅ | 50Hz 前瞻检查 + IDLE/EXEC/SLOWDOWN/BRAKE 降级阶梯（gtest + RMUC 仿真验证）|
| RMUC 仿真 | ✅ | submodule 提供 |
| 真车适配 | 🟡 | 需替换 LIO 输入话题 + map→odom 全局定位 |

详细的 bug 与改进项见仓外审计文档（不入版本控制）。

---

## 开发路线（短中期）

### 短期（修 bug + 接缝）

- 在线建图模式接入全局规划器（完成 `OnlineMapProxy`）
- ~~局部规划器 trajectory swap 原子化（修 race）~~ ✅ 已完成（`TrajSnapshot` 只读快照交换）
- 点云分支补 raycast clearing，避免动障永久残留
- 高度过滤参数从 yaml 实际生效（透传到 `SensorProcessor`）

### 中期（功能完整化）

- 多层占用图（ground / body / head），实现狗洞净空判定
- 时间衰减体素 + LiDAR 视锥清除
- 可通行性融合层（occupancy + slope + clearance + roughness）
- launch 改 condition-based 替代固定延迟

### 长期

- MINCO 时间-轨迹联合优化
- MPC 升级到带不等式硬约束的 QP（OSQP）
- ROS 2 LifecycleNode 改造，支持运行时重配置
- 算法层 gtest 覆盖

---

## 调试小贴士

- 高程更新频率应匹配车速：5 m/s 推荐 ≥ 15 Hz
- 点云密度过大会拖慢 ESDF（20 Hz timer），可在 LIO 输出端做下采样
- Zenoh 模式下若发现节点互相发现不到，先确认 `rmw_zenohd` router 已启动
- 若 RViz 看到 `/sdf_map/esdf` 不连续：通常是滑窗边界，正常现象
- 真车上线前：删除 `bringup.launch.py` 里的静态 `map→odom` TF，接入 AMCL 或视觉定位
- 若局部规划器 NaN 崩溃：通常是 `solveQuintic` 输入 waypoints 过于贴近，检查 A* 输出去重

---

## 致谢

- [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) — kinodynamic A* / EDTEnvironment 设计参考
- [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner) — MINCO 轨迹优化思路
- [STVL](https://github.com/SteveMacenski/spatio_temporal_voxel_layer) — 时间衰减体素栅格设计参考
- [ROG-Map](https://github.com/hku-mars/ROG-Map) — ESDF / 膨胀图实现参考
- [Small Point-LIO](https://github.com/Aurora-UJS/small_point_lio) — LIO 定位
- [Eclipse Zenoh](https://zenoh.io/) — ROS 2 中间件

---

## License

BSD-3-Clause
