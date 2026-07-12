# Sentry Nav 26

面向 RoboMaster 哨兵机器人的模块化自主导航栈，基于 **ROS 2 Jazzy** + **Gazebo Harmonic**，提供从 LIO 定位、在线建图、全局规划到局部轨迹生成与跟踪的完整闭环。

---

## 特性

- **LIO 定位**：基于 [Small Point-LIO](https://github.com/Aurora-UJS/small_point_lio)，输出高频 `/Odometry`（含 ESKF twist）与配准点云 `/cloud_registered`
- **在线 2D 建图**：log-odds 占用栅格 + 法向量地形分类（墙面成障、≤30° 坡面放行）+ Felzenszwalb 两遍 ESDF + 占据超时自愈（`plan_env`）
- **全局规划**：JPS（Jump Point Search）+ 静态先验 PGM 地图（`sentry_global_planner`）
- **局部规划**：Kinodynamic A* + 多项式 Shot 直达（`path_searching`）；ESDF 验收（薄障碍免疫）+ 失败部分路径打捞 + 搜索时间预算
- **轨迹优化**：MINCO 五次多项式 + L-BFGS（NLopt），平滑/避障/可行性/单向逆行多目标
- **轨迹跟踪**：50 Hz LDLT-MPC（10 步预测 horizon）+ 开阔地自转扫描；窄道/坡道对齐模式（迟滞触发，停自转、航向对齐、恒低速直穿）
- **执行安全与脱困**：50 Hz 轨迹前瞻监控 + IDLE/EXEC/SLOWDOWN/BRAKE 降级阶梯（BRAKE 静置降级出口，无吸收态）；卡滞虚拟障碍注入侧移 + 窄道爬行兜底
- **指令架构**：导航栈输出世界系速度 `/cmd_vel_world`，底盘侧（仿真 `chassis_cmd_node` / 真车电控 MCU）用高频陀螺 yaw 旋转到机体系——自转下 LIO yaw 有 50~70ms 龄期，不可用于 50Hz 旋转
- **可通行性标注层**：离线 `.trav.yaml` 三态标注（free/obstacle/oneway），JPS 软代价 + A*/MINCO 硬约束，RViz 绘制插件（OPT-IN 默认关）
- **仿真环境**：RMUC 2025 赛场（[`rm_sim_26`](https://github.com/Neomelt/rm_sim_26)）+ Livox 雷达 / IMU 桥接；支持 RGL（NVIDIA GPU 高精度 Mid360）与内置 `gpu_lidar` 双模式（`lidar_mode`）
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
│   │   ├── kinodynamic_astar.cpp    # 二阶动力学 A* + OBVP shot + ESDF 验收/部分路径打捞
│   │   ├── polynomial_solver.hpp    # 三/四次方程解析求根
│   │   └── test/                    # A* 几何回归 gtest（薄带/死锁复刻/预算/豁免）
│   │
│   ├── sentry_global_planner/   # JPS 全局路径搜索
│   │   ├── global_planner_node.cpp
│   │   ├── jps_searcher.hpp         # JPS 算法实现（含 oneway 软惩罚）
│   │   └── global_map.hpp           # PGM 加载 + 静态 ESDF + OnlineMapProxy
│   │
│   ├── sentry_local_planner/    # 局部规划 + 控制 + 执行安全
│   │   ├── sentry_local_planner_node.cpp   # 顶层节点（10Hz 按需重规划 + 50Hz 控制）
│   │   ├── chassis_cmd_node.cpp            # 仿真"底盘固件"：/cmd_vel_world → 陀螺 yaw 旋转 → /cmd_vel
│   │   ├── minco_trajectory.cpp            # MINCO 轨迹生成 + L-BFGS 优化
│   │   ├── trajectory_tracker.cpp          # MPC 跟踪 + 窄道/坡道对齐模式 + 爬行兜底
│   │   ├── mpc_controller.hpp              # LDLT 求解的二阶 MPC
│   │   ├── planner_fsm.hpp                 # IDLE/EXEC/SLOWDOWN/BRAKE 纯函数状态机
│   │   ├── safety_monitor.hpp              # 执行中轨迹前瞻安全检查
│   │   ├── chassis_yaw_estimator.hpp       # 陀螺积分 yaw + LIO 零漂校准（|wz| 门控）
│   │   └── test/                           # FSM / 安全监控 / 对齐跟踪 / yaw 估计 gtest
│   │
│   ├── sentry_trav_rviz_plugin/ # RViz 可通行性标注绘制插件（导出 .trav.yaml）
│   │
│   └── sentry_msgs/             # 自定义 msg/srv/action 模板
│
├── docs/                        # TRAVERSABILITY.md / REVIEW.md / 台沿死锁根因取证报告与图表
├── scripts/                     # podman-dev.sh 容器工作流 + diag/ 量化诊断脚本
├── test/                        # launch 集成测试占位
├── debug_planner.sh             # GDB 调试脚本
├── Containerfile                # Podman 开发镜像定义
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
   │       ├─ 点云法向量分类 → log-odds occupancy (raycast 清除 + 占据超时)
   │       └─ 滑窗 ESDF (Felzenszwalb 两遍可分离 EDT)
   │
   ├──► sentry_global_planner (JPS, 静态先验地图 + 可通行性标注)
   │       └─ /global_path  (nav_msgs/Path)
   │
   └──► sentry_local_planner
           ├─ 订阅 /Odometry, /goal_pose, /global_path
           ├─ KinodynamicAstar 在 EDTEnvironment 上搜索 (10Hz 按需重规划)
           ├─ MINCO 五次轨迹 + L-BFGS 优化 (粗 8ms / 精 12ms)
           └─ MPC / 对齐模式跟踪 (50Hz) ──►  /cmd_vel_world (odom 系)
                                                │
                                                ▼
                              chassis_cmd_node (陀螺 yaw 旋转; 真车=电控 MCU)
                                                └──►  /cmd_vel (机体系)
```

---

## 关键话题与接口

### 输入
| 话题 | 类型 | 来源 |
|---|---|---|
| `/Odometry` | `nav_msgs/Odometry` | small_point_lio（含 ESKF twist 速度反馈）|
| `/cloud_registered` | `sensor_msgs/PointCloud2` | small_point_lio |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz 2D Goal Pose |
| `/chassis/imu` | `sensor_msgs/Imu` | 底盘 IMU（chassis_cmd_node 陀螺 yaw 积分）|

### 中间
| 话题 | 类型 | 发布者 | 订阅者 |
|---|---|---|---|
| `/global_path` | `nav_msgs/Path` | sentry_global_planner | sentry_local_planner |
| `/sdf_map/occupancy` | `sensor_msgs/PointCloud2` | plan_env | RViz 调试 |
| `/sdf_map/esdf` | `sensor_msgs/PointCloud2` | plan_env | RViz 调试 |

### 输出
| 话题 | 类型 | 用途 |
|---|---|---|
| `/cmd_vel_world` | `geometry_msgs/Twist` | 世界系速度指令（规划器输出，`controller.cmd_frame: world` 默认）|
| `/cmd_vel` | `geometry_msgs/Twist` | 机体系底盘命令（chassis_cmd_node 陀螺 yaw 旋转后转发）|
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
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

> **必须显式指定 build type**：colcon 默认不传任何优化标志（等效 `-O0`），实测 A* 单次扩展 35~56ms、直接吃光重规划时间预算。`scripts/podman-dev.sh build` 已内置该参数。

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
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
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
# 仿真用 mid360.yaml（config/ 仅有 mid360.yaml 与 unilidar_l2.yaml）
timeout 15 ros2 run small_point_lio small_point_lio_node \
    --ros-args --params-file install/small_point_lio/share/small_point_lio/config/mid360.yaml
```

---

## 关键参数

### 局部规划器（`sentry_local_planner/config/planner_params.yaml`）

| 项 | 默认 | 说明 |
|---|---|---|
| `controller.frequency` | 50 Hz | MPC 控制频率 |
| `replan.frequency` | 10 Hz | 重规划**检查**频率；按需触发（无轨迹/偏差超限/目标移动/周期刷新），非每拍全量重规划 |
| `search.max_vel / max_acc` | 3.0 / 2.0 | 动力学上限（acc 3.0 在 3 rad/s 自转下实测侧翻，降 2.0）|
| `search.accept_clearance` | 0.28 m | A* ESDF 验收阈值 = robot_radius − res/2；阈值链 0.28 > warn 0.20 > hard 0.15 |
| `search.max_search_time_ms` | 40 ms | A* 时间预算：目标不可达时防全域搜索拖死重规划 |
| `safety.margin_warn / margin_hard` | 0.20 / 0.15 m | 执行监控分级阈值（warn 降速 / hard 迫近才刹停）|
| `controller.corridor_enter/exit_dist` | 0.55 / 0.80 m | 窄道对齐模式进/出迟滞（前瞻 min-ESDF）|
| `controller.slope_enter/exit_deg` | 7° / 4° | 坡道对齐模式进/出迟滞（机身倾角）|
| `replan.crawl_fail_threshold` | 5 | 规划连败此数后开启窄道爬行兜底 |
| `mpc.horizon` | 10 | 预测步数（horizon = N · dt = 0.2s）|
| `mpc.q_pos / q_vel / r_acc` | 10 / 1 / 0.1 | MPC 状态/输入权重 |
| `minco_opt.max_time_ms` | 20.0 | L-BFGS 总预算 (ms)，节点内 /1000（粗 40% / 精 60% → 8ms / 12ms）|
| `minco_opt.dist0` / `dist0_vel_k` | 0.30 / 0.0 | footprint ESDF 软间隙（有余地时主动留 0.3m）+ 速度相关裕度（dist0_eff = dist0 + k·\|v\|，0=关闭）|
| `minco_opt.lambda_oneway` | 8.0 | 单向标注区逆行软代价权重 |

### 全局规划器（`sentry_global_planner/config/global_planner_params.yaml`）

| 项 | 默认 | 说明 |
|---|---|---|
| `global_map.mode` | `prior` | `prior` = 加载 PGM；`online` = 用 plan_env 实时 ESDF（**已接入 JPS，待端到端验证**） |
| `jps.safety_dist` | 0.3 m | ESDF 小于此值视为不可通行 |
| `jps.esdf_weight` | 2.0 | ESDF 代价权重，越大越远离障碍 |
| `jps.waypoint_spacing` | 1.0 m | 简化后路径点最小间距 |
| `jps.oneway_penalty` | 50.0 | ONEWAY 标注区逆向软惩罚（代价乘子增量，不硬挡保连通）|

### 在线建图（`plan_env`，由 launch 注入）

| 项 | 默认 | 说明 |
|---|---|---|
| `sdf_map.resolusion_` | 0.05 m | 体素分辨率（**注意 yaml 里键名拼写**）|
| `sdf_map.local_update_range_x/y` | 8.0 m | 局部窗口半径（planner_params.yaml 注入值；代码 declare 默认 3.0）|
| `sdf_map.obstacles_inflation` | 0.05 m | 膨胀半径，仅做噪声平滑（车体安全由规划器 ESDF 验收负责）|
| `sdf_map.max_slope_deg` | 30° | 法向量坡度判定：≤30° 斜面放行，墙面/立面 \|n.z\|≈0 成障 |
| `sdf_map.normal_voxel_leaf / normal_k` | 0.08 m / 10 | 法向量估计前体素降采样 + kNN 邻域点数 |
| `sdf_map.cloud_min/max_height` | -0.2 / 0.15 m | 相对机身 z 的判障带：只取机身高度附近表面，高处平台/桥面不在带内 |
| `sdf_map.logodds_*` | hit 0.85 / miss -0.4 | 贝叶斯更新参数 |
| `sdf_map.occ_timeout` | 3.5 s | 占据超时清除：超时未再命中的格子视为空闲（自转幽灵障碍自愈）|

---

## 可通行性标注层（方向性 / 单向）

先验 PGM 与在线占用图都是「无方向」的双态地图——能走就双向都能走。但雷达架设较高，扫不到台阶**立面**，会把「只能下不能上」的台阶当成双向平地。**可通行性标注层**离线补一张与地图同坐标系的 `*.trav.yaml`，给规划器引入第三态：

| 类型 | 含义 |
|---|---|
| `free` | 普通可通行（无约束）|
| `obstacle` | 人工补充障碍（双向禁止，补感知盲区）|
| `oneway` | 单向可通行（带允许行进朝向 `direction_deg` + 容差锥 `tolerance_deg`）|

- **帧约定**：世界/odom 系、米，origin 取 `rmuc_2025.yaml` 的 origin；规划器起点 = 地图原点，故**无需额外 TF**。
- **方向语义**：`direction_deg` = 允许行进航向 `atan2(dy,dx)` 度数；放行 ⇔ `dot(unit(travel), dir) >= cos(tolerance_deg)`；默认 `90°`（cos=0）= 前向半球，逆向被挡。
- **各层 gate**：全局 JPS = **软方向代价 + obstacle 叠加**（不硬挡 oneway，保持全局图连通完整）；局部 Kinodynamic A* + MINCO = **硬方向约束**（基于边，逆向直接剪除）。
- **OPT-IN，默认关**：出厂 `rmuc_2025.trav.yaml` 为 `regions: []` → `loadFromYaml()` 返回 false → 本层 disabled → 行为与今天完全一致。仅当指向含有效 region 的 yaml 时才生效：

```bash
ros2 launch sentry_bringup bringup.launch.py \
    trav_yaml:=$HOME/sentry_nav_26/install/sentry_bringup/share/sentry_bringup/map/rmuc_2025.trav.yaml
```

> 标注用 RViz `Publish Point`（`/clicked_point` 回读坐标）手编 yaml，或用 `sentry_trav_rviz_plugin` 可视化绘制导出。完整数据模型、格式、gate 机制与已知限制见 **[docs/TRAVERSABILITY.md](docs/TRAVERSABILITY.md)**。

---

## 当前实现状态

| 模块 | 完成度 | 备注 |
|---|---|---|
| LIO 定位 | ✅ | 通过 submodule |
| 在线 2D occupancy + ESDF | ✅ | log-odds + Felzenszwalb |
| 点云法向量地形分类 | ✅ | 法向量 \|n.z\| 判墙面/坡面（≤30° 放行），替代高程差判定——自转误配准保朝向不保位移，法向量对误配准鲁棒（幽灵障碍 31→0）；raycast 清除 + 占据超时自愈 |
| 多层占用（狗洞净空检测）| ❌ | 未实现 |
| 时间衰减体素 (STVL) | ❌ | log-odds 永久累积，无衰减 |
| 视锥清除 | ❌ | 未实现 |
| 可通行性融合层 | ❌ | 未实现（occupancy + slope + clearance + roughness 在线融合）|
| 可通行性标注层（单向/方向）| ✅ | `.trav.yaml`（free/obstacle/oneway）贯穿 JPS 软代价 / A*+MINCO 硬约束；RViz 标注插件；OPT-IN 默认关（`regions: []` 即 disabled），详见 `docs/TRAVERSABILITY.md` |
| 全局规划 (JPS, 静态先验) | ✅ | 完整可用 |
| 全局规划在线模式 | 🟡 | `OnlineMapProxy` 已接入 JPS（global_planner_node.cpp `online` 分支），但未端到端验证；直接启用会因缺 `sdf_map.*` 参数崩溃，需先补参数接线 |
| Kinodynamic A* | ✅ | 二阶动力学 + OBVP shot；ESDF 验收（accept_clearance 0.28，薄障碍免疫）+ near_end 失败继续搜 + 部分路径打捞 + 搜索时间预算 + 可通行性标注检查（obstacle/oneway）|
| MINCO 轨迹优化 | ✅ | 五次多项式 + L-BFGS；footprint + Lipschitz 剪枝 + 单向逆行软代价 |
| MPC 跟踪 | ✅ | LDLT，软约束 |
| 执行中轨迹安全监控 | ✅ | 50Hz 前瞻检查 + IDLE/EXEC/SLOWDOWN/BRAKE 降级阶梯 + BRAKE 静置超时出口（gtest + RMUC 仿真验证）|
| 窄道/坡道对齐模式 | ✅ | 前瞻 min-ESDF/倾角触发（迟滞），停自转+航向对齐+纯追踪恒速直穿；爬行兜底 |
| 世界系指令 + 底盘 yaw 下沉 | ✅ | `/cmd_vel_world` → chassis_cmd_node 高频陀螺积分 yaw 旋转（真车由电控 MCU 替代同一逻辑）|
| RMUC 仿真 | ✅ | submodule 提供 |
| 真车适配 | 🟡 | 需替换 LIO 输入话题 + map→odom 全局定位 |

详细的 bug 与改进项见 `docs/REVIEW.md`（代码审计报告）。

---

## 开发路线（短中期）

### 短期（修 bug + 接缝）

- 端到端验证在线建图模式（`OnlineMapProxy` 已接入 JPS，待跑通 + 调参）
- ~~局部规划器 trajectory swap 原子化（修 race）~~ ✅ 已完成（`TrajSnapshot` 只读快照交换）
- ~~点云分支补 raycast clearing~~ ✅ 已完成（法向量分类 + raycast + 占据超时）
- 标注图补高地南沿缺口（x 6.5~12.4 段，真值有 0.2m 台阶，JPS 可能直穿）
- 幽灵格注入恢复时限、0.70m 东坡道通行的整机回归测试

### 中期（功能完整化）

- 多层占用图（ground / body / head），实现狗洞净空判定
- 时间衰减体素 + LiDAR 视锥清除
- 可通行性融合层（occupancy + slope + clearance + roughness）
- launch 改 condition-based 替代固定延迟

### 长期

- MINCO 时间-轨迹联合优化
- MPC 升级到带不等式硬约束的 QP（OSQP）
- ROS 2 LifecycleNode 改造，支持运行时重配置
- ~~算法层 gtest 覆盖~~ ✅ A* 几何回归 / FSM / 安全监控 / 对齐跟踪 / yaw 估计已覆盖（46 用例）；整机 launch 集成测试待补

---

## 调试小贴士

- 高程更新频率应匹配车速：5 m/s 推荐 ≥ 15 Hz
- 点云密度过大会拖慢 ESDF（20 Hz timer），可在 LIO 输出端做下采样
- Zenoh 模式下若发现节点互相发现不到，先确认 `rmw_zenohd` router 已启动
- 若 RViz 看到 `/sdf_map/esdf` 不连续：通常是滑窗边界，正常现象
- 真车上线前：删除 `sim_test.launch.py` 里的静态 `map→odom` TF（bringup 经 sim_test 引入），接入 AMCL 或视觉定位；`chassis_cmd_node` 由电控 MCU 固件替代
- 若局部规划器 NaN 崩溃：通常是 `solveQuintic` 输入 waypoints 过于贴近，检查 A* 输出去重
- 机器人趴窝先 grep 阶段归因日志：规划失败必有 `Plan stage A*/sampling/MINCO` 与 A* 失败原因（budget/open-empty/allocate）输出，静默无日志本身就是 bug
- 性能异常（规划超预算）先确认 build type：`grep -- -O build/<pkg>/CMakeFiles/*/flags.make`，colcon 默认无优化标志

---

## 致谢

- [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) — kinodynamic A* / EDTEnvironment 设计参考
- [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner) — MINCO 轨迹优化思路
- [rose_navigation](https://github.com/hyheiyue/rose_navigation) — clearance 软代价、局部性重规划、窄道 turtle 模式与路径切向 yaw 参考等哨兵导航实践参考
- [中国科学技术大学 RoboWalker 哨兵导航技术报告 (RM2025)](https://bbs.robomaster.com/article/803740) — 哨兵导航整体方案与工程实践参考
- [STVL](https://github.com/SteveMacenski/spatio_temporal_voxel_layer) — 时间衰减体素栅格设计参考
- [ROG-Map](https://github.com/hku-mars/ROG-Map) — ESDF / 膨胀图实现参考
- [Small Point-LIO](https://github.com/Aurora-UJS/small_point_lio) — LIO 定位
- [Eclipse Zenoh](https://zenoh.io/) — ROS 2 中间件

---

## License

BSD-3-Clause
