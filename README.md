# Sentry Nav 26

A modular navigation system built with ROS 2 Jazzy and Nav2 for autonomous robot navigation.

Overview

This workspace contains packages for simulation, localization, mapping, navigation, and system integration, designed for modular development and testing with Gazebo Harmonic and Nav2.

## 需求分析

|场景|需求|
|--|--|
|15°斜坡|坡度计算、可通行性判断|
狗洞 550×200mm|悬空障碍检测、通道高度检查
起伏路段 2m×70cm|地形连续性分析
时间衰减体素|	STVL|	处理移动障碍物（其他机器人、人）
视锥清除	|STVL	|加速清除应该空但没观测到的区域
高程图	|新增|	坡度+起伏分析
净空检测|	新增|	狗洞通过判断
膨胀图	|ROG-Map	|安全距离
ESDF	|ROG-Map|	轨迹优化梯度 (可简化为2.5D)
|
## 项目架构图
``` zsh
┌──────────────────────────────────────────────────────────────────┐
│                        传感器输入                                 │
│         激光雷达 (Livox等) + IMU + 轮式编码器 (可选)              │
└───────────────────────────┬──────────────────────────────────────┘
                            │
                            ▼
┌──────────────────────────────────────────────────────────────────┐
│               Small Point-LIO (定位+稀疏地图)                     │
│  输出:                                                           │
│    • nav_msgs/Odometry        ← 位姿 (高频)                      │
│    • sensor_msgs/PointCloud2  ← 配准后点云 (用于感知)             │
│    • 内部 ikd-tree            ← SLAM用的稀疏地图                  │
└───────────────────────────┬──────────────────────────────────────┘
                            │
         ┌──────────────────┴──────────────────┐
         ▼                                     ▼
┌─────────────────────────┐      ┌─────────────────────────────────┐
│  时间衰减体素栅格        │      │         高程图                   │
│  (用于规划的稠密地图)    │      │   (坡度/狗洞/起伏分析)           │
│                         │      │                                 │
│  • 接收 cloud_registered │      │  • 接收 cloud_registered        │
│  • 动态障碍物衰减        │      │  • 提取地面高度                  │
│  • 输出 3D占用栅格       │      │  • 计算可通行性                  │
└───────────┬─────────────┘      └───────────────┬─────────────────┘
            │                                    │
            └──────────────┬─────────────────────┘
                           ▼
┌──────────────────────────────────────────────────────────────────┐
│                    可通行性地图 (Traversability)                  │
│  融合: 障碍物占用 + 坡度 + 净空高度 + 起伏度                       │
└───────────────────────────┬──────────────────────────────────────┘
                            ▼
┌──────────────────────────────────────────────────────────────────┐
│                    膨胀图 + 简化ESDF                              │
└───────────────────────────┬──────────────────────────────────────┘
                            ▼
┌──────────────────────────────────────────────────────────────────┐
│                         规划层                                    │
│            A* (可通行性代价) → 轨迹优化                           │
└──────────────────────────────────────────────────────────────────┘
```

## 核心数据结构
``` c++
// 高程图 - 每个2D格子存储地面高度信息
struct ElevationCell {
    float height;           // 地面高度 (m)
    float variance;         // 高度不确定性
    float slope;            // 局部坡度 (rad)
    float roughness;        // 粗糙度
    uint8_t confidence;     // 观测置信度
};

// 简化3D占用图 - 只保留机器人相关的高度层
// 例如: 地面层、机器人高度层、头顶层
struct MultiLayerOccupancy {
    bool ground_layer;      // 0-10cm，检测地面障碍
    bool body_layer;        // 10-35cm，机器人本体高度
    bool head_layer;        // 35-50cm，检测悬空障碍(狗洞顶)
    float clearance;        // 头顶净空高度
};

// 可通行性
struct TraversabilityCell {
    float cost;             // 综合代价 [0, 1], 1=不可通行
    bool passable;          // 是否可通行
    uint8_t terrain_type;   // 地形类型: 平地/斜坡/起伏/狗洞
};
```

``` cpp
class SentryNavigation {
public:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // 从 Small Point-LIO 获取位姿
        current_pose_ = poseFromMsg(msg);
    }
    
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 从 Small Point-LIO 获取配准后的点云
        PointCloud cloud;
        pcl::fromROSMsg(*msg, cloud);
        
        // 更新感知地图 (全部在线!)
        temporal_voxel_grid_.update(cloud, current_pose_, current_time_);
        elevation_map_.update(cloud, current_pose_);
        
        // 分析可通行性
        traversability_.analyze(
            temporal_voxel_grid_,
            elevation_map_
        );
        
        // 更新规划用的地图
        inflation_map_.update(traversability_.getGrid());
    }
    
private:
    // Small Point-LIO 输出
    Pose current_pose_;
    
    // 在线感知 (无需预建地图)
    TemporalVoxelGrid temporal_voxel_grid_;  // 时间衰减体素
    ElevationMap elevation_map_;              // 高程图
    TraversabilityAnalyzer traversability_;   // 可通行性
    
    // 规划地图
    InflationMap inflation_map_;
};
```

## 关键算法
1. 斜坡检测与可通行性
``` cpp
float computeSlopeCost(const ElevationMap& map, int x, int y) {
    // 计算局部坡度 (用周围格子的高度差)
    float dz_dx = (map(x+1,y).height - map(x-1,y).height) / (2 * resolution);
    float dz_dy = (map(x,y+1).height - map(x,y-1).height) / (2 * resolution);
    float slope = atan(sqrt(dz_dx*dz_dx + dz_dy*dz_dy));
    
    // 15度 ≈ 0.26 rad, 设最大可通行坡度
    constexpr float MAX_SLOPE = 0.30;  // ~17度
    if (slope > MAX_SLOPE) return 1.0;  // 不可通行
    
    return slope / MAX_SLOPE;  // 线性代价
}
```
2. 狗洞检测
``` cpp
bool canPassDogHole(const MultiLayerOccupancy& occ, 
                    float robot_height, float robot_width) {
    // 狗洞条件: 地面通畅 + 有足够净空
    // 你的机器人: 假设高度 ~35cm, 宽度 ~45cm
    // 狗洞: 高200mm, 宽550mm
    
    if (occ.ground_layer) return false;  // 地面有障碍
    if (occ.clearance < robot_height + 0.05) return false;  // 净空不足
    
    // 还需要检查宽度方向的通道
    return true;
}
```

3. 起伏路段检测

``` cpp
float evaluateUndulation(const ElevationMap& map, 
                         const Path& path_segment) {
    // 沿路径计算高度变化
    float max_step = 0;
    for (size_t i = 1; i < path_segment.size(); i++) {
        float dh = abs(map.at(path_segment[i]).height - 
                       map.at(path_segment[i-1]).height);
        max_step = std::max(max_step, dh);
    }
    
    // 70cm起伏，检查单步高度变化是否在轮子能力范围内
    constexpr float MAX_STEP_HEIGHT = 0.08;  // 8cm单步
    return (max_step > MAX_STEP_HEIGHT) ? 1.0 : max_step / MAX_STEP_HEIGHT;
}
```

## 包结构示例

``` zsh
sentry_nav/
├── sentry_msgs/           # 消息定义
├── sentry_elevation_map/  # 高程图 (参考ETH elevation_mapping简化)
├── sentry_occupancy/      # 多层占用图
├── sentry_traversability/ # 可通行性分析
├── sentry_planner/        # A* + 轨迹优化
└── sentry_bringup/        # 启动文件
```

## Prerequisites

ROS 2 Jazzy
Gazebo Harmonic
Ubuntu 24.04 (Noble Numbat)

## Installation
``` zsh
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-gz-tools-vendor ros-jazzy-gz-sim-vendor ros-jazzy-ros-gz
cd ~/sentry_nav_26
git submodule update --init --recursive
colcon build --symlink-install

# 安装 NLopt 开发包
sudo apt update
sudo apt install libnlopt-dev libnlopt-cxx-dev

# 验证安装
pkg-config --modversion nlopt

# Usage
source ~/sentry_nav_26/install/setup.bash
ros2 launch sentry_bringup bringup.launch.py
```


祝你的 RM 哨兵导航系统开发顺利！如果遇到问题随时可以问喵～

几个小提醒：

高程图更新频率要和车速匹配，5m/s 大概需要 10-20Hz
狗洞检测记得考虑机器人的姿态变化（过狗洞时可能需要低头）
起伏路段可以用样条插值平滑高程图，减少噪声影响
加油喵！期待看到你的成果！🎮🤖

关键数据流

```cpp
// 每帧更新流程
void SentryNavigation::update(const PointCloud& cloud, const Pose& pose) {
    // 1. STVL风格：时间衰减 + 视锥清除
    temporal_voxel_grid_.decayVoxels(current_time);
    temporal_voxel_grid_.clearFrustum(sensor_frustum);
    temporal_voxel_grid_.insertPoints(cloud);
    
    // 2. 高程图更新
    elevation_map_.update(cloud, pose);
    
    // 3. 可通行性分析
    traversability_.analyze(
        temporal_voxel_grid_,  // 障碍物
        elevation_map_         // 地形
    );
    
    // 4. ROG-Map风格：膨胀 + 可选ESDF
    inflation_map_.update(traversability_.getOccupancyGrid());
    // esdf_.update(inflation_map_);  // 如果需要轨迹优化
}
```
具体实现建议
1. 用STVL处理动态环境
``` cpp
// 直接用或参考STVL的时间衰减模型
struct TemporalVoxel {
    double timestamp;      // 标记时间
    uint8_t occupancy;     // 占用状态
    
    bool isExpired(double now, double decay_time) const {
        return (now - timestamp) > decay_time;
    }
};
```

2. 增加高程层

```cpp
// 在STVL基础上加高程信息
struct TerrainCell {
    float ground_height;     // 地面高度
    float clearance;         // 头顶净空
    float slope;             // 坡度
    bool passable;           // 可通行性
};
```
3. 简化ESDF为2.5D
``` cpp
// 不需要完整3D ESDF，只在地面层计算
class SimplifiedESDF2D {
    // 只计算 (x, y) 到最近障碍物的距离
    // 比完整3D ESDF快很多
    float getDistance(float x, float y) const;
    Vec2f getGradient(float x, float y) const;
};
```
最终包结构

``` zsh
sentry_nav/
├── sentry_perception/          # 感知层
│   ├── temporal_voxel_grid.hpp   # 借鉴STVL
│   ├── elevation_map.hpp         # 高程图
│   └── traversability.hpp        # 可通行性
├── sentry_map/                 # 地图层
│   ├── inflation_map.hpp         # 借鉴ROG-Map
│   └── esdf_2d.hpp               # 简化ESDF
├── sentry_planner/             # 规划层
│   ├── astar_search.hpp
│   └── trajectory_opt.hpp
└── sentry_bringup/             # 启动
```
开发路线
Week 1-2: temporal_voxel_grid + elevation_map (感知基础)
Week 3: traversability (狗洞/坡度分析)
Week 4: inflation_map + esdf_2d (地图层)
Week 5-6: astar_search + trajectory_opt (规划层)
这样既有STVL的动态场景处理能力，又有ROG-Map的轨迹优化支持，还加入了你需要的地形分析喵！🎮✨


License
BSD-3-Clause

重启small_point_lio节点
``` zsh
source install/setup.zsh && timeout 15 ros2 run small_point_lio small_point_lio_node --ros-args --params-file install/small_point_lio/share/small_point_lio/config/simulation.yaml 2>&1
```