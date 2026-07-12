#pragma once
/**
 * MapCore: 纯数据 + 算法层 (不依赖 ROS)
 *
 * 持有地图缓冲区，提供坐标转换、ESDF 计算、环形缓冲区管理。
 */

#include <plan_env/map_data.hpp>

class MapCore
{
public:
    MapCore() = default;
    ~MapCore() = default;

    // 初始化缓冲区 (由 SDFMap::initMap 调用)
    void initBuffers(const MappingParameters &mp, int buffer_size);

    // ==================== 坐标转换 ====================
    void posToIndex(const Eigen::Vector2d &pos, Eigen::Vector2i &id) const;
    void indexToPos(const Eigen::Vector2i &id, Eigen::Vector2d &pos) const;
    int toAddress(const Eigen::Vector2i &id) const;
    int toAddress(int x, int y) const;
    void boundIndex(Eigen::Vector2i &id) const;
    bool isInMap(const Eigen::Vector2d &pos) const;
    bool isInMap(const Eigen::Vector2i &idx) const;

    // ==================== 查询接口 ====================
    double getDistance(const Eigen::Vector2d &pos) const;
    double getDistanceByIndex(const Eigen::Vector2i &idx) const;
    int getInflateOccupancy(Eigen::Vector2d pos) const;
    double getResolution() const { return mp_.resolution_; }
    void getRegion(Eigen::Vector2d &ori, Eigen::Vector2d &size) const;
    Eigen::Vector2d getMapOrigin() const { return mp_.map_origin_; }
    void getSurroundPts(const Eigen::Vector2d &pos, Eigen::Vector2d pts[2][2], Eigen::Vector2d &diff) const;

    // ==================== 缓冲区操作 ====================
    void resetBuffer(Eigen::Vector2d min_pos, Eigen::Vector2d max_pos);
    void slideMapTo(const Eigen::Vector2d &new_center);
    void clearRingSlice(int dim, int from, int to);
    void raycast(const Eigen::Vector2i &start, const Eigen::Vector2i &end);
    // now: 当前传感器时刻 (s)，用于 occ_timeout_ 龄期门控
    void thresholdLogodds(double now);
    // 卡滞脱困: 注入虚拟障碍块（雷达扫不到的小坎等本体感知障碍），occ_timeout 自动过期
    void addVirtualObstacle(const Eigen::Vector2d &pos, double radius, double now);
    // 静态先验层查询（世界坐标，独立于滑窗环形缓冲）
    bool isStaticOccupied(const Eigen::Vector2d &pos) const;

    // ==================== ESDF ====================
    void updateESDF2d();

    // ==================== 全局地图切换 ====================
    void setLocalMap(int num);

    // 数据直接暴露给 SensorProcessor 和 SDFMap (友元级访问)
    MappingData md_;
    MappingParameters mp_;

private:
    template <typename F_get_val, typename F_set_val>
    void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
