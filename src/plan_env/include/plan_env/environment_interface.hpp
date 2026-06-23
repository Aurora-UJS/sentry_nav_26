#pragma once
/**
 * EnvironmentInterface: 环境查询抽象接口
 *
 * 所有规划/优化模块通过此接口查询地图，不直接依赖 SDFMap 或 EDTEnvironment 具体实现。
 */

#include <Eigen/Eigen>

namespace sentry_nav
{

class EnvironmentInterface
{
public:
    virtual ~EnvironmentInterface() = default;

    // 距离查询
    virtual double getDistance(const Eigen::Vector2d &pos) = 0;
    virtual int getInflateOccupancy(const Eigen::Vector2d &pos) = 0;
    virtual bool isInMap(const Eigen::Vector2d &pos) = 0;

    // ESDF 梯度查询
    virtual void evaluateEDTWithGrad(const Eigen::Vector2d &pos, double time,
                                     double &dist, Eigen::Vector2d &grad) = 0;
    virtual double evaluateCoarseEDT(const Eigen::Vector2d &pos, double time) = 0;
    virtual bool hasDynamicObjects() const = 0;

    // 地图信息
    virtual void getMapRegion(Eigen::Vector2d &ori, Eigen::Vector2d &size) = 0;
    virtual double getResolution() = 0;

    // 静态可通行性标注层 (free/obstacle/one-way)。默认实现为"无标注 → 全部放行",
    // 这样未挂载标注层的环境实现无需改动。世界/odom 坐标查询。
    //   getTravType: 0=free, 1=obstacle, 2=oneway
    virtual int getTravType(const Eigen::Vector2d &pos)
    {
        (void)pos;
        return 0;
    }
    // travel_dir 无需归一化; FREE→true, OBSTACLE→false, ONEWAY→在容差锥内才 true
    virtual bool isDirectionAllowed(const Eigen::Vector2d &pos, const Eigen::Vector2d &travel_dir)
    {
        (void)pos;
        (void)travel_dir;
        return true;
    }
    // 命中 ONEWAY 格则填 dir(单位允许方向)+cos_tol 并返回 true; 否则返回 false (供 MINCO 软代价)
    virtual bool getOnewayConstraint(const Eigen::Vector2d &pos, Eigen::Vector2d &dir, double &cos_tol)
    {
        (void)pos;
        dir.setZero();
        cos_tol = -1.0;
        return false;
    }
};

} // namespace sentry_nav
