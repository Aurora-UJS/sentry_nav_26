#pragma once
/**
 * SensorProcessor: 传感器数据处理 (不直接依赖 ROS 消息类型)
 *
 * 接收解析后的点云/激光数据，执行坡度检测、高程图更新、障碍标记。
 */

#include <plan_env/map_core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class SensorProcessor
{
public:
    SensorProcessor() = default;

    void setMapCore(MapCore *core) { core_ = core; }

    /**
     * 处理 3D 点云 (PointCloud2 → 高程图 + 坡度检测 + 障碍标记)
     * @param cloud_3d  odom 系下的 3D 点云
     * @param robot_pos 机器人 2D 位置
     * @param robot_z   机器人 Z 高度
     * @param now_sec   点云时刻 (s)，用于占据超时龄期
     */
    void processCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud_3d,
                      const Eigen::Vector2d &robot_pos, double robot_z, double now_sec);

    /**
     * 处理 2D 激光 (LaserScan → 障碍标记 + 光线追踪)
     * @param laser     odom 系下的 2D 激光点云
     * @param robot_pos 机器人 2D 位置
     * @param now_sec   激光时刻 (s)，用于占据超时龄期
     */
    void processLaser(const pcl::PointCloud<pcl::PointXY> &laser,
                      const Eigen::Vector2d &robot_pos, double now_sec);

private:
    MapCore *core_ = nullptr;
};
