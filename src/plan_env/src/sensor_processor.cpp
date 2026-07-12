#include "plan_env/sensor_processor.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

void SensorProcessor::processCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud_3d,
                                   const Eigen::Vector2d &robot_pos, double robot_z, double now_sec)
{
    if (!core_) return;
    auto &md = core_->md_;
    auto &mp = core_->mp_;

    double min_h = -0.1, max_h = 1.0;
    // 这些参数应该通过 MapCore 的 mp_ 传入，暂时用 mp_ 中的坡度参数
    // 实际 min_h/max_h 在 SDFMap::initMap 中从 ROS 参数读取并设置

    min_h += robot_z;
    max_h += robot_z;

    core_->slideMapTo(robot_pos);

    // 清空局部高程图
    Eigen::Vector2i local_min, local_max;
    core_->posToIndex(-mp.local_update_range_ + robot_pos, local_min);
    core_->posToIndex(mp.local_update_range_ + robot_pos, local_max);
    core_->boundIndex(local_min);
    core_->boundIndex(local_max);
    int buf_size = (int)md.elevation_buffer_.size();
    for (int x = local_min(0); x <= local_max(0); ++x)
        for (int y = local_min(1); y <= local_max(1); ++y) {
            int addr = core_->toAddress(x, y);
            if (addr >= 0 && addr < buf_size)
                md.elevation_buffer_[addr] = std::numeric_limits<float>::quiet_NaN();
        }

    // Pass 1: 填充高程图
    for (const auto &pt : cloud_3d.points)
    {
        if (pt.z < min_h - 0.5 || pt.z > max_h + 0.5)
            continue;

        Eigen::Vector2d p2d(pt.x, pt.y);
        Eigen::Vector2d devi = p2d - robot_pos;
        if (devi.norm() < 0.6)
            continue;
        if (fabs(devi(0)) >= mp.local_update_range_(0) || fabs(devi(1)) >= mp.local_update_range_(1))
            continue;

        Eigen::Vector2i idx;
        core_->posToIndex(p2d, idx);
        if (!core_->isInMap(idx))
            continue;

        int addr = core_->toAddress(idx);
        if (addr < 0 || addr >= buf_size)
            continue;
        float &elev = md.elevation_buffer_[addr];
        if (std::isnan(elev) || pt.z < elev)
            elev = pt.z;
    }

    // Pass 1.5: 光线追踪清除自由空间
    Eigen::Vector2i sensor_idx;
    core_->posToIndex(robot_pos, sensor_idx);
    for (const auto &pt : cloud_3d.points)
    {
        if (pt.z < min_h || pt.z > max_h)
            continue;

        Eigen::Vector2d p2d(pt.x, pt.y);
        Eigen::Vector2d devi = p2d - robot_pos;
        if (devi.norm() < 0.6)
            continue;

        Eigen::Vector2i hit_idx;
        core_->posToIndex(p2d, hit_idx);
        if (!core_->isInMap(hit_idx))
            continue;

        core_->raycast(sensor_idx, hit_idx);
    }

    // Pass 2: 坡度/台阶/悬空障碍检测
    int inf_step = ceil(mp.obstacles_inflation_ / mp.resolution_);

    for (const auto &pt : cloud_3d.points)
    {
        if (pt.z < min_h || pt.z > max_h)
            continue;

        Eigen::Vector2d p2d(pt.x, pt.y);
        Eigen::Vector2d devi = p2d - robot_pos;
        if (devi.norm() < 0.6)
            continue;
        if (fabs(devi(0)) >= mp.local_update_range_(0) || fabs(devi(1)) >= mp.local_update_range_(1))
            continue;

        Eigen::Vector2i idx;
        core_->posToIndex(p2d, idx);
        if (!core_->isInMap(idx))
            continue;

        int addr = core_->toAddress(idx);
        if (addr < 0 || addr >= buf_size)
            continue;
        float center_h = md.elevation_buffer_[addr];
        if (std::isnan(center_h))
            continue;

        bool is_slope_ok = true;
        float max_gradient = 0;
        for (int di = -1; di <= 1; ++di)
        {
            for (int dj = -1; dj <= 1; ++dj)
            {
                if (di == 0 && dj == 0) continue;
                Eigen::Vector2i nb(idx(0) + di, idx(1) + dj);
                if (!core_->isInMap(nb)) continue;
                int nb_addr = core_->toAddress(nb);
                if (nb_addr < 0 || nb_addr >= buf_size) continue;
                float nb_h = md.elevation_buffer_[nb_addr];
                if (std::isnan(nb_h)) continue;

                float dh = fabs(center_h - nb_h);
                float dist = sqrt(di*di + dj*dj) * mp.resolution_;
                float gradient = atan2(dh, dist);

                max_gradient = std::max(max_gradient, gradient);

                if (dh > mp.step_height_max_)
                    is_slope_ok = false;
            }
        }

        if (max_gradient > mp.max_slope_rad_)
            is_slope_ok = false;

        if (pt.z - center_h > 0.15)
            is_slope_ok = false;

        if (!is_slope_ok)
        {
            for (int dx = -inf_step; dx <= inf_step; ++dx)
            {
                for (int dy = -inf_step; dy <= inf_step; ++dy)
                {
                    Eigen::Vector2d p2d_inf(pt.x + dx * mp.resolution_,
                                            pt.y + dy * mp.resolution_);
                    Eigen::Vector2i inf_pt;
                    core_->posToIndex(p2d_inf, inf_pt);
                    if (!core_->isInMap(inf_pt))
                        continue;
                    int inf_addr = core_->toAddress(inf_pt);
                    md.logodds_buffer_[inf_addr] += (float)mp.logodds_hit_;
                    if (md.logodds_buffer_[inf_addr] > (float)mp.logodds_max_)
                        md.logodds_buffer_[inf_addr] = (float)mp.logodds_max_;
                    md.last_hit_time_[inf_addr] = (float)now_sec;
                }
            }
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
