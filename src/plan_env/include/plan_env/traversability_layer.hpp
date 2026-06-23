#pragma once
/**
 * TraversabilityLayer: 静态可通行性标注层 (free / obstacle / one-way+direction)
 *
 * 动机:
 *   先验 PGM 地图只有 free/obstacle 两态。某些区域 (如台阶) 由于雷达架设较高
 *   在线感知看不到立面 → 地图里是"可通行", 但物理上只能"下不能上"。本层在
 *   世界坐标 (= odom, 起点在原点约定) 引入第三态 ONEWAY(带允许行进方向) 以及
 *   人工补充的 OBSTACLE, 供全局 JPS / 局部 Kinodynamic A* / MINCO 共享查询。
 *
 * 坐标系:
 *   标注以世界/米为单位, 与先验地图同坐标 (origin = rmuc_2025.yaml 的 origin)。
 *   规划器全程在 odom 中工作且约定起点=地图原点, 故标注无需额外 TF 即对齐。
 *
 * 数据流:
 *   离线编写 rmuc_2025.trav.yaml (regions 多边形 + 类型 + 方向) →
 *   loadFromYaml() 栅格化到内部网格 → O(1) 查询。
 *   全局与局部规划器各自加载同一个 yaml, 行为一致。
 *
 * 方向语义 (ONEWAY):
 *   direction_deg 为"允许行进朝向" (世界系, atan2(dy,dx) 角度)。
 *   tolerance_deg 为半张角 (默认 90°): 行进方向 t 与允许方向 d 满足
 *   dot(unit(t), d) >= cos(tolerance_deg) 才放行。
 *   90° → cos=0 → 允许前向半球 (逆向被挡)。
 */

#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <memory>
#include <cstdint>

namespace sentry_nav
{

enum class TravType : uint8_t
{
    FREE = 0,      // 普通可通行 (无约束)
    OBSTACLE = 1,  // 人工补充障碍 (双向禁止)
    ONEWAY = 2     // 单向可通行 (带允许方向 + 容差)
};

// 单元格标注 (栅格化后每格存一份)
struct TravCell
{
    TravType type = TravType::FREE;
    Eigen::Vector2d dir = Eigen::Vector2d::Zero();  // 允许行进单位方向 (仅 ONEWAY 有意义)
    double cos_tol = 0.0;                           // cos(半张角); 放行条件 dot(unit(t),dir) >= cos_tol
};

class TraversabilityLayer
{
public:
    using Ptr = std::shared_ptr<TraversabilityLayer>;

    // 原始标注区域 (供可视化 / RViz 插件回读)
    struct Region
    {
        std::string id;
        TravType type = TravType::FREE;
        std::vector<Eigen::Vector2d> polygon;  // 世界坐标顶点 (>=3)
        double dir_deg = 0.0;                  // 允许行进朝向 (ONEWAY)
        double tol_deg = 90.0;                 // 半张角
    };

    // 从 yaml 加载并栅格化。失败返回 false 且本层保持 disabled (查询全部按 FREE/放行)。
    bool loadFromYaml(const std::string &yaml_path);

    bool enabled() const { return enabled_; }
    size_t regionCount() const { return regions_.size(); }
    const std::vector<Region> &regions() const { return regions_; }

    // --- 世界系查询 (越界或未启用 → FREE) ---
    TravType getType(const Eigen::Vector2d &world_pos) const;

    // ONEWAY 约束查询: 命中 ONEWAY 格则填 dir(单位)+cos_tol 返回 true; 否则 false。
    // 供 MINCO 软代价/梯度使用。
    bool getOnewayConstraint(const Eigen::Vector2d &world_pos,
                             Eigen::Vector2d &dir, double &cos_tol) const;

    // 行进方向是否被允许。travel_dir 无需归一化; 近零长度视为无方向信息 → 放行。
    //   FREE     → true
    //   OBSTACLE → false
    //   ONEWAY   → dot(unit(travel_dir), dir) >= cos_tol
    bool isDirectionAllowed(const Eigen::Vector2d &world_pos,
                            const Eigen::Vector2d &travel_dir) const;

    // 栅格元信息 (可视化 / 调试)
    double resolution() const { return resolution_; }
    Eigen::Vector2d origin() const { return origin_; }
    int width() const { return width_; }
    int height() const { return height_; }

private:
    bool inGrid(int gx, int gy) const { return gx >= 0 && gx < width_ && gy >= 0 && gy < height_; }
    bool worldToGrid(const Eigen::Vector2d &p, int &gx, int &gy) const;
    static bool pointInPolygon(const Eigen::Vector2d &p, const std::vector<Eigen::Vector2d> &poly);
    void rasterize();

    bool enabled_ = false;
    double resolution_ = 0.05;
    double resolution_inv_ = 20.0;
    Eigen::Vector2d origin_ = Eigen::Vector2d::Zero();  // 栅格 (0,0) 左下角世界坐标
    int width_ = 0, height_ = 0;
    double default_tol_deg_ = 90.0;

    std::vector<Region> regions_;
    std::vector<TravCell> grid_;        // width_*height_, 行主序: idx = gx*height_ + gy (与 PriorMap 一致)
    TravCell free_sentinel_;            // 越界返回引用用
};

}  // namespace sentry_nav
