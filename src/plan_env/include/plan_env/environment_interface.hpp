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
};

} // namespace sentry_nav
