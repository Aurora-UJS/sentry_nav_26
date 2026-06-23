# 可通行性标注层 (Traversability Annotation Layer)

> 给先验地图补充「方向性可通行」语义的离线标注层。
> 出厂默认关闭（OPT-IN）：不写标注 = 行为与今天完全一致。

---

## 1. 动机：为什么需要它

先验 PGM 地图（`rmuc_2025.yaml`）和在线占用栅格都只有 **free / obstacle 两态**，且都是「无方向」的——能走就双向都能走。但实战里有两类先验知识它们表达不出来：

1. **雷达看不见的立面**：哨兵雷达架设较高，扫不到台阶**立面**，只扫到上下两个平台。于是占用图把台阶当成一片连续「平地」，标成可通行。可物理上车只能从高处往低处 **下**，不能从低处往高处 **上**——这是一个**单向**通道，地图却说它双向可走。
2. **感知盲区里的真实障碍**：玻璃栏杆、悬空横杆下方、反光面等，在线感知漏检，但确实挡路，需要人工补一笔 **obstacle**。

可通行性标注层就是把这些**离线已知**的先验，写进一个与地图同坐标系的 `*.trav.yaml`，让全局/局部规划器查询时遵守。它引入第三态：

| 类型 | 含义 | 方向语义 |
|---|---|---|
| `free` | 普通可通行，无约束 | — |
| `obstacle` | 人工补充障碍，**双向禁止** | — |
| `oneway` | **单向**可通行，带允许行进方向 + 容差 | 见 §4 |

---

## 2. 数据模型

标注层栅格化为一张与先验地图同分辨率的网格，每格存一个 `TravCell`：

```
TravType  type      // FREE=0 / OBSTACLE=1 / ONEWAY=2
Vector2d  dir        // 允许行进的单位方向（仅 ONEWAY 有意义）
double    cos_tol     // cos(半张角)；放行条件 dot(unit(travel), dir) >= cos_tol
```

加载流程：离线编写 `rmuc_2025.trav.yaml`（若干多边形 region + 类型 + 方向）→ `loadFromYaml()` 把每个多边形栅格化进网格 → 运行期 O(1) 查询。全局规划器与局部规划器**各自加载同一个 yaml**，因此行为一致。

对外查询接口（经 `EnvironmentInterface`，规划器直接调用，无需 plumbing）：

```cpp
int  getTravType(pos);                              // 0=free / 1=obstacle / 2=oneway
bool isDirectionAllowed(pos, travel_dir);           // free->true, obstacle->false, oneway->锥判定
bool getOnewayConstraint(pos, dir&, cos_tol&);      // 命中 oneway 格时填 dir+cos_tol 返回 true
```

越界或未启用 → `getTravType` 返回 FREE、`isDirectionAllowed` 返回 true（**全部按放行**）。

---

## 3. YAML 格式

文件：`src/sentry_bringup/map/rmuc_2025.trav.yaml`（随 `map/` 目录一起 install 到 share）。

```yaml
map:
  resolution: 0.05          # 栅格分辨率 (m)，与先验地图一致
  origin: [-3.58, -9.44]    # 栅格 (0,0) 左下角世界坐标，与 rmuc_2025.yaml 的 origin 一致
  width: 0                  # 0 = 自动：由所有 region 包围盒推导
  height: 0                 # 0 = 自动（若 width/height>0 且给出 origin，则用声明值）

default_tolerance_deg: 90.0 # region 未写 tolerance_deg 时的默认半张角

regions:                    # 空列表 [] => 本层 disabled（出厂状态）
  - id: step_south_descend  # 任意可读标识
    type: oneway            # free | obstacle | oneway
    direction_deg: -90.0    # 允许行进朝向（世界系航向角 = atan2(dy,dx) 度数）
    tolerance_deg: 90.0     # 半张角（度）；oneway 才有意义
    polygon:                # >=3 个世界坐标米顶点；首尾自动闭合，顺/逆时针均可
      - [ 1.20, -2.40 ]
      - [ 2.60, -2.40 ]
      - [ 2.60, -3.10 ]
      - [ 1.20, -3.10 ]
```

字段说明：

| 字段 | 适用类型 | 说明 |
|---|---|---|
| `id` | 全部 | 可读标识，仅用于日志/可视化 |
| `type` | 全部 | `free` / `obstacle` / `oneway` |
| `direction_deg` | oneway | 允许行进朝向，世界系 `atan2(dy,dx)` 的度数（见 §4） |
| `tolerance_deg` | oneway | 半张角（度），缺省取 `default_tolerance_deg` |
| `polygon` | 全部 | 顶点列表，**>=3** 个 `[x, y]` 世界坐标米；不足 3 个的 region 被跳过 |

规则：

- **区域可重叠，后写覆盖先写**——`regions` 的顺序即优先级，列表里靠后的 region 覆盖与它重叠的格子。
- `obstacle` 忽略 `direction_deg` / `tolerance_deg`。
- **空 `regions: []` → `loadFromYaml()` 返回 false → 本层 disabled**，所有查询按放行，行为与未启用完全一致（这是出厂状态，保证 OPT-IN）。

---

## 4. 帧约定与方向语义

### 坐标系（无额外 TF）

标注全部以**世界 / odom 系、米**为单位，与先验地图同坐标（`origin` 取 `rmuc_2025.yaml` 的 origin）。规划器全程在 **odom** 中工作，且约定**起点 = 地图原点**（start-at-origin），因此 `map` 与 `odom` 视为恒等，标注无需任何额外 TF 即与实时位姿对齐。

> 这也意味着 v1 假设 `map -> odom` 恒等。长程匹配漂移会让标注与真实场地错位——见 §8 已知限制。

### direction_deg

`direction_deg` 是**允许行进的朝向**，世界系航向角，等于 `atan2(dy, dx)` 换算成度：

```
  0   = +X (东 / east)        90  = +Y (北 / north)
 -90  = -Y (南 / south)       180 = -X (西 / west)
```

例：一段台阶从北往南是「下坡」，要表达「只能往南下、不能往北上」，就取 `direction_deg: -90.0`。

### 容差锥 (tolerance cone)

放行判定是一个以 `dir` 为轴的**圆锥**：

```
travel_dir 单位化为 t，dir 为 d（单位向量），cos_tol = cos(tolerance_deg)
放行 ⇔ dot(t, d) >= cos_tol
```

| tolerance_deg | cos_tol | 含义 |
|---|---|---|
| 90° | 0.0 | **前向半球**：只要行进方向沿箭头有正分量就放行，逆向被挡（**默认**） |
| 45° | ≈0.707 | 仅在以箭头为轴的 ±45° 锥内放行 |
| 0° | 1.0 | 只允许严格同向 |

零长度 `travel_dir`（原地不动）视为**无方向信息 → 放行**。

---

## 5. 各规划层如何 gate

设计上**全局软、局部硬**——既保证全局图连通完整，又在局部做权威方向约束。

### 全局 JPS（软方向代价 + obstacle 叠加）

- `obstacle` 区域：直接叠加为不可通行（与占用图同等对待）。
- `oneway` 区域：**不硬挡**，而是加一个**方向软代价**——逆着允许方向走会被加价，但路径仍可经过。这样 JPS 图保持**连通完整**，不会因为单向约束把全局图切断导致搜索失败。
- 全局只给一条「倾向遵守方向」的引导路径，真正的硬约束交给局部。

### 局部 Kinodynamic A*（硬方向 gate）

- A* 是**基于边（运动基元）**的搜索，每条边天然带行进方向，适合做硬约束。
- 扩展节点 / shot 落在 `obstacle` 格 → 直接判死（`getTravType == 1`）。
- 落在 `oneway` 格 → 用该边的行进方向查 `isDirectionAllowed`，**逆向边直接剪掉**（硬 gate）。

### MINCO 轨迹优化（硬方向约束）

- 轨迹采样点落在 `oneway` 格时，用 `getOnewayConstraint` 取出 `dir + cos_tol`，对**逆向运动**施加约束/代价，把轨迹挤回允许方向的锥内。
- 与局部 A* 一致，是**权威硬约束**——最终下发的轨迹保证满足方向语义。

> 锥判定数学全层一致：`dot(unit(travel), dir) >= cos(tolerance_deg)`。

---

## 6. 标注工作流

两种方式，任选其一：

### A. RViz 标注插件（推荐）

`sentry_trav_rviz_plugin`（标注插件）可在 RViz 里**可视化绘制**多边形、设置类型/方向/容差，导出成 `*.trav.yaml`。适合大批量、所见即所得的标注。

### B. 手编 yaml + Publish Point 读坐标

1. RViz 里加载先验地图，确认 `Fixed Frame` 与标注同系（odom / map）。
2. 用 RViz 顶栏的 **Publish Point** 工具点取场地上的点，话题 `/clicked_point` 会回显该点的世界坐标：
   ```bash
   ros2 topic echo /clicked_point
   ```
3. 依次点取多边形各顶点，把读到的 `x, y` 填进 `polygon`。
4. `oneway` 的 `direction_deg`：沿「允许行进方向」点两个点，`atan2(y2-y1, x2-x1)` 换算成度即可（或直接按 §4 的方位对照表估）。

---

## 7. 如何启用

本层 **OPT-IN**，靠规划器侧的参数指向一个 yaml 路径：

- launch 参数：`trav_yaml`
- 节点参数：`traversability.yaml_path`

```bash
# 指向标注文件即启用
ros2 launch sentry_bringup bringup.launch.py \
    trav_yaml:=$HOME/sentry_nav_26/install/sentry_bringup/share/sentry_bringup/map/rmuc_2025.trav.yaml
```

- **路径为空 / 文件缺失 / `regions: []` → 本层 disabled**，所有查询按放行，行为与今天完全一致。
- 仅当指向一个**含有效 region** 的 yaml 时，`loadFromYaml()` 才返回 true、本层挂载生效。

> 出厂的 `rmuc_2025.trav.yaml` 是 `regions: []`（INERT），即使被指向也保持关闭——直到用户填入真实标注。

---

## 8. v1 已知限制

- **仅静态地图**：标注是离线先验，运行期不更新。场地临时变化（被推倒的障碍、新增结构）不会反映。
- **假设 `map -> odom` 恒等**：依赖 start-at-origin 约定，没有全局重定位。**长程 LIO 匹配漂移**会让标注与真实场地逐渐错位（标注框「飘」到错误位置），台阶/障碍判定随之失准。
- **多边形栅格化按格心判定**：分辨率 0.05m 下，区域边界有半格量级的量化误差（正常近似，非缺陷）；标注框请略放大覆盖目标。
- **无高度维度**：标注是 2D 的，无法表达「某高度以下可钻、以上是横杆」这类净空语义——那属于多层占用图（见 README roadmap），与本层正交。

**未来工作**：接入 ICP / 视觉重定位估计 `map -> odom`，让标注随定位刷新而对齐，解除 start-at-origin 的硬假设。

---

## 9. 相关文件

| 文件 | 作用 |
|---|---|
| `src/sentry_bringup/map/rmuc_2025.trav.yaml` | 标注模板（出厂 INERT），随 `map/` install 到 share |
| `src/plan_env/include/plan_env/traversability_layer.hpp` | 标注层数据结构与查询接口（**锁定契约**） |
| `src/plan_env/src/traversability_layer.cpp` | yaml 加载 + 栅格化 + 查询实现 |
| `src/plan_env/include/plan_env/environment_interface.hpp` | 规划器共享查询入口（3 个虚函数，默认放行） |
| `src/plan_env/include/plan_env/edt_environment.hpp` | `EDTEnvironment` 挂载标注层并 override 查询 |
