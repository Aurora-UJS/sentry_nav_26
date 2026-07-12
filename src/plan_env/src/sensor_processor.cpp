#include "plan_env/sensor_processor.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <unordered_map>

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>

/**
 * 障碍分类: 法向量坡度判定 (借鉴 rose_navigation bin_map)
 *
 * 为什么用法向量而不是相邻格高差梯度 (旧实现):
 * 刚体误配准 (自转下 LIO 位姿龄期) 保持平面的"朝向"仅偏差 1~2°，但让逐点
 * z 位移出现 ±10cm 级误差。5cm 基线上 17° 坡度阈值只允许 dh 1.5cm，被噪声
 * 淹没 —— 实测自转时地板以 ~3200 次/s 被误标记 (98% 由梯度检查触发)。
 * 法向量在 kNN 邻域上拟合平面朝向，对该噪声天然鲁棒: 地板/缓坡 |n.z|→1，
 * 墙面/陡坡 |n.z|→0。
 *
 * z 带 [robot_z + cloud_min_h, robot_z + cloud_max_h] 只取机身高度附近的
 * 表面参与判定: 高处平台面/桥面不在带内 (不挡路也不标记)，台阶沿的竖直
 * 侧面在带内 |n.z|≈0 → 障碍。带随机身 z 移动，上坡过程持续成立。
 */

void SensorProcessor::processCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud_3d,
                                   const Eigen::Vector2d &robot_pos, double robot_z, double now_sec)
{
    if (!core_) return;
    auto &md = core_->md_;
    auto &mp = core_->mp_;

    core_->slideMapTo(robot_pos);

    // 1. 体素降采样: 均衡点密度，控制 kNN 法向量的开销
    pcl::PointCloud<pcl::PointXYZ>::Ptr ds(new pcl::PointCloud<pcl::PointXYZ>);
    {
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud_3d.makeShared());
        const float leaf = (float)mp.normal_voxel_leaf_;
        vg.setLeafSize(leaf, leaf, leaf);
        vg.filter(*ds);
    }
    if (ds->points.empty())
        return;

    // 2. kNN 法向量 (OMP)
    pcl::PointCloud<pcl::Normal> normals;
    {
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(ds);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        ne.setSearchMethod(tree);
        ne.setKSearch(mp.normal_k_);
        ne.compute(normals);
    }

    const double min_z = robot_z + mp.cloud_min_h_;
    const double max_z = robot_z + mp.cloud_max_h_;

    Eigen::Vector2i sensor_idx;
    core_->posToIndex(robot_pos, sensor_idx);

    // 3. 带内点: 光线追踪清自由空间 + 按 2D 格聚合 |n.z|
    //    key = x * map_voxel_num_y + y (非环形地址，仅本帧内聚合用)
    struct CellAcc { float sum_cos = 0.f; int count = 0; };
    std::unordered_map<int, CellAcc> acc;
    acc.reserve(4096);

    for (size_t i = 0; i < ds->points.size(); ++i)
    {
        const auto &pt = ds->points[i];
        if (pt.z < min_z || pt.z > max_z)
            continue;

        Eigen::Vector2d p2d(pt.x, pt.y);
        Eigen::Vector2d devi = p2d - robot_pos;
        if (devi.norm() < 0.6)
            continue;

        Eigen::Vector2i idx;
        core_->posToIndex(p2d, idx);
        if (!core_->isInMap(idx))
            continue;

        core_->raycast(sensor_idx, idx);

        if (fabs(devi(0)) >= mp.local_update_range_(0) || fabs(devi(1)) >= mp.local_update_range_(1))
            continue;

        const auto &n = normals.points[i];
        if (!std::isfinite(n.normal_z))
            continue;

        auto &a = acc[idx(0) * mp.map_voxel_num_(1) + idx(1)];
        a.sum_cos += fabs(n.normal_z);
        a.count += 1;
    }

    // 4. 格级判定: 点数足够且平均法向量偏离竖直 → 障碍
    const float cos_thresh = (float)cos(mp.max_slope_rad_);
    const int inf_step = (int)ceil(mp.obstacles_inflation_ / mp.resolution_);

    for (const auto &kv : acc)
    {
        const CellAcc &a = kv.second;
        if (a.count < mp.normal_count_thresh_)
            continue;
        if (a.sum_cos / a.count >= cos_thresh)
            continue; // 地板/可通行坡面

        Eigen::Vector2i idx(kv.first / mp.map_voxel_num_(1), kv.first % mp.map_voxel_num_(1));
        for (int dx = -inf_step; dx <= inf_step; ++dx)
            for (int dy = -inf_step; dy <= inf_step; ++dy)
            {
                Eigen::Vector2i inf_pt(idx(0) + dx, idx(1) + dy);
                if (!core_->isInMap(inf_pt))
                    continue;
                int addr = core_->toAddress(inf_pt);
                md.logodds_buffer_[addr] += (float)mp.logodds_hit_;
                if (md.logodds_buffer_[addr] > (float)mp.logodds_max_)
                    md.logodds_buffer_[addr] = (float)mp.logodds_max_;
                md.last_hit_time_[addr] = (float)now_sec;
            }
    }

    Eigen::Vector2d bound_max_pos = mp.local_update_range_ + robot_pos;
    Eigen::Vector2d bound_min_pos = -mp.local_update_range_ + robot_pos;

    core_->posToIndex(bound_max_pos, md.local_bound_max_);
    core_->posToIndex(bound_min_pos, md.local_bound_min_);

    core_->boundIndex(md.local_bound_min_);
    core_->boundIndex(md.local_bound_max_);

    core_->thresholdLogodds(now_sec);

    md.esdf_need_update_ = true;
    md.update_num_ += 1;
}

void SensorProcessor::processLaser(const pcl::PointCloud<pcl::PointXY> &laser,
                                   const Eigen::Vector2d &robot_pos, double now_sec)
{
    if (!core_) return;
    auto &md = core_->md_;
    auto &mp = core_->mp_;

    core_->slideMapTo(robot_pos);

    // 光线追踪
    Eigen::Vector2i sensor_idx;
    core_->posToIndex(robot_pos, sensor_idx);
    for (size_t i = 0; i < laser.points.size(); ++i)
    {
        Eigen::Vector2d p2d(laser.points[i].x, laser.points[i].y);
        Eigen::Vector2d devi = p2d - robot_pos;
        if (devi.norm() < 0.2)
            continue;

        Eigen::Vector2i hit_idx;
        core_->posToIndex(p2d, hit_idx);
        if (!core_->isInMap(hit_idx))
            continue;

        core_->raycast(sensor_idx, hit_idx);
    }

    // 障碍标记
    int inf_step = ceil(mp.obstacles_inflation_ / mp.resolution_);

    for (size_t i = 0; i < laser.points.size(); ++i)
    {
        Eigen::Vector2d p2d(laser.points[i].x, laser.points[i].y);
        Eigen::Vector2d devi = p2d - robot_pos;
        if (devi.norm() < 0.2)
            continue;
        if (fabs(devi(0)) < mp.local_update_range_(0) && fabs(devi(1)) < mp.local_update_range_(1))
        {
            for (int x = -inf_step; x <= inf_step; ++x)
                for (int y = -inf_step; y <= inf_step; ++y)
                {
                    Eigen::Vector2d p2d_inf(laser.points[i].x + x * mp.resolution_,
                                            laser.points[i].y + y * mp.resolution_);
                    Eigen::Vector2i inf_pt;
                    core_->posToIndex(p2d_inf, inf_pt);
                    if (!core_->isInMap(inf_pt))
                        continue;
                    int idx_inf = core_->toAddress(inf_pt);
                    md.logodds_buffer_[idx_inf] += (float)mp.logodds_hit_;
                    if (md.logodds_buffer_[idx_inf] > (float)mp.logodds_max_)
                        md.logodds_buffer_[idx_inf] = (float)mp.logodds_max_;
                    md.last_hit_time_[idx_inf] = (float)now_sec;
                }
        }
    }

    core_->posToIndex(mp.local_update_range_ + robot_pos, md.local_bound_max_);
    core_->posToIndex(-mp.local_update_range_ + robot_pos, md.local_bound_min_);

    core_->boundIndex(md.local_bound_min_);
    core_->boundIndex(md.local_bound_max_);

    core_->thresholdLogodds(now_sec);

    md.esdf_need_update_ = true;
    md.update_num_ += 1;
}
