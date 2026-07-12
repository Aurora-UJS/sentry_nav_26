#include "plan_env/map_core.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

using namespace std;

void MapCore::initBuffers(const MappingParameters &mp, int buffer_size)
{
    mp_ = mp;
    md_.occupancy_buffer_neg = vector<char>(buffer_size, 0);
    md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
    md_.distance_buffer_ = vector<double>(buffer_size, 10000);
    md_.distance_buffer_neg_ = vector<double>(buffer_size, 10000);
    md_.distance_buffer_all_ = vector<double>(buffer_size, 10000);
    md_.tmp_buffer1_ = vector<double>(buffer_size, 0);
    md_.elevation_buffer_ = vector<float>(buffer_size, std::numeric_limits<float>::quiet_NaN());
    md_.slope_obstacle_buffer_ = vector<char>(buffer_size, 0);
    md_.logodds_buffer_ = vector<float>(buffer_size, 0.0f);
    md_.last_hit_time_ = vector<float>(buffer_size, MappingData::kNeverHit);

    md_.local_bound_min_ = Eigen::Vector2i::Zero();
    md_.local_bound_max_ = Eigen::Vector2i::Zero();
    md_.ring_offset_ = Eigen::Vector2i::Zero();
    md_.has_odom_ = false;
    md_.has_cloud_ = false;
    md_.esdf_need_update_ = false;
    md_.update_num_ = 0;
    md_.max_esdf_time_ = 0.0;
}

void MapCore::posToIndex(const Eigen::Vector2d &pos, Eigen::Vector2i &id) const
{
    for (int i = 0; i < 2; ++i)
        id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

void MapCore::indexToPos(const Eigen::Vector2i &id, Eigen::Vector2d &pos) const
{
    for (int i = 0; i < 2; ++i)
        pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

int MapCore::toAddress(const Eigen::Vector2i &id) const
{
    int bx = (id(0) + md_.ring_offset_(0)) % mp_.map_voxel_num_(0);
    int by = (id(1) + md_.ring_offset_(1)) % mp_.map_voxel_num_(1);
    if (bx < 0) bx += mp_.map_voxel_num_(0);
    if (by < 0) by += mp_.map_voxel_num_(1);
    return bx * mp_.map_voxel_num_(1) + by;
}

int MapCore::toAddress(int x, int y) const
{
    int bx = (x + md_.ring_offset_(0)) % mp_.map_voxel_num_(0);
    int by = (y + md_.ring_offset_(1)) % mp_.map_voxel_num_(1);
    if (bx < 0) bx += mp_.map_voxel_num_(0);
    if (by < 0) by += mp_.map_voxel_num_(1);
    return bx * mp_.map_voxel_num_(1) + by;
}

void MapCore::boundIndex(Eigen::Vector2i &id) const
{
    Eigen::Vector2i id1;
    id1(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
    id1(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
    id = id1;
}

bool MapCore::isInMap(const Eigen::Vector2i &idx) const
{
    if (idx(0) < 0 || idx(1) < 0)
        return false;
    if (idx(0) > mp_.map_voxel_num_(0) - 1 || idx(1) > mp_.map_voxel_num_(1) - 1)
        return false;
    return true;
}

bool MapCore::isInMap(const Eigen::Vector2d &pos) const
{
    if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4)
        return false;
    if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4)
        return false;
    return true;
}

double MapCore::getDistance(const Eigen::Vector2d &pos) const
{
    Eigen::Vector2i id;
    posToIndex(pos, id);
    Eigen::Vector2i bounded = id;
    const_cast<MapCore*>(this)->boundIndex(bounded);
    return md_.distance_buffer_all_[toAddress(bounded)];
}

double MapCore::getDistanceByIndex(const Eigen::Vector2i &idx) const
{
    Eigen::Vector2i id = idx;
    const_cast<MapCore*>(this)->boundIndex(id);
    return md_.distance_buffer_all_[toAddress(id)];
}

int MapCore::getInflateOccupancy(Eigen::Vector2d pos) const
{
    if (!isInMap(pos))
        return -1;
    Eigen::Vector2i id;
    posToIndex(pos, id);
    return int(const_cast<MappingData&>(md_).is_occupancy(toAddress(id)));
}

void MapCore::getRegion(Eigen::Vector2d &ori, Eigen::Vector2d &size) const
{
    ori = mp_.map_origin_;
    size = mp_.map_size_;
}

void MapCore::getSurroundPts(const Eigen::Vector2d &pos, Eigen::Vector2d pts[2][2], Eigen::Vector2d &diff) const
{
    Eigen::Vector2d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector2d::Ones();
    Eigen::Vector2i idx;
    Eigen::Vector2d idx_pos;

    posToIndex(pos_m, idx);
    indexToPos(idx, idx_pos);
    diff = (pos - idx_pos) * mp_.resolution_inv_;

    for (int x = 0; x < 2; ++x)
        for (int y = 0; y < 2; ++y)
        {
            Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
            Eigen::Vector2d current_pos;
            indexToPos(current_idx, current_pos);
            pts[x][y] = current_pos;
        }
}

void MapCore::setLocalMap(int num)
{
    if (num > md_.global_map_num || num < 0)
        return;
    md_.current_global_map = num;
}

void MapCore::resetBuffer(Eigen::Vector2d min_pos, Eigen::Vector2d max_pos)
{
    Eigen::Vector2i min_id, max_id;
    posToIndex(min_pos, min_id);
    posToIndex(max_pos, max_id);
    boundIndex(min_id);
    boundIndex(max_id);

    for (int x = min_id(0); x <= max_id(0); ++x)
        for (int y = min_id(1); y <= max_id(1); ++y)
        {
            md_.occupancy_buffer_inflate_[toAddress(x, y)] = 0;
            md_.distance_buffer_[toAddress(x, y)] = 10000;
        }
}

void MapCore::clearRingSlice(int dim, int from, int to)
{
    int N0 = mp_.map_voxel_num_(0);
    int N1 = mp_.map_voxel_num_(1);
    int buf_size = N0 * N1;

    auto clear_addr = [&](int addr) {
        if (addr < 0 || addr >= buf_size) return;
        md_.occupancy_buffer_inflate_[addr] = 0;
        md_.occupancy_buffer_neg[addr] = 0;
        md_.distance_buffer_[addr] = 10000;
        md_.distance_buffer_neg_[addr] = 10000;
        md_.distance_buffer_all_[addr] = 10000;
        md_.tmp_buffer1_[addr] = 0;
        md_.elevation_buffer_[addr] = std::numeric_limits<float>::quiet_NaN();
        md_.slope_obstacle_buffer_[addr] = 0;
        md_.logodds_buffer_[addr] = 0.0f;
        md_.last_hit_time_[addr] = MappingData::kNeverHit;
    };

    if (dim == 0) {
        for (int x = from; x < to; ++x) {
            int lx = ((x % N0) + N0) % N0;
            for (int y = 0; y < N1; ++y) {
                int bx = (lx + md_.ring_offset_(0)) % N0;
                int by = (y + md_.ring_offset_(1)) % N1;
                if (bx < 0) bx += N0;
                if (by < 0) by += N1;
                clear_addr(bx * N1 + by);
            }
        }
    } else {
        for (int y = from; y < to; ++y) {
            int ly = ((y % N1) + N1) % N1;
            for (int x = 0; x < N0; ++x) {
                int bx = (x + md_.ring_offset_(0)) % N0;
                int by = (ly + md_.ring_offset_(1)) % N1;
                if (bx < 0) bx += N0;
                if (by < 0) by += N1;
                clear_addr(bx * N1 + by);
            }
        }
    }
}

void MapCore::slideMapTo(const Eigen::Vector2d &new_center)
{
    Eigen::Vector2d new_origin = new_center - mp_.map_size_ * 0.5;

    Eigen::Vector2i shift;
    for (int i = 0; i < 2; ++i)
        shift(i) = (int)round((new_origin(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);

    int threshold = (int)(mp_.map_voxel_num_(0) * 0.1);
    if (abs(shift(0)) < threshold && abs(shift(1)) < threshold)
        return;

    int N0 = mp_.map_voxel_num_(0);
    int N1 = mp_.map_voxel_num_(1);

    if (shift(0) > 0) {
        clearRingSlice(0, N0 - shift(0), N0);
    } else if (shift(0) < 0) {
        clearRingSlice(0, 0, -shift(0));
    }

    md_.ring_offset_(0) = ((md_.ring_offset_(0) + shift(0)) % N0 + N0) % N0;

    if (shift(1) > 0) {
        clearRingSlice(1, N1 - shift(1), N1);
    } else if (shift(1) < 0) {
        clearRingSlice(1, 0, -shift(1));
    }

    md_.ring_offset_(1) = ((md_.ring_offset_(1) + shift(1)) % N1 + N1) % N1;

    for (int i = 0; i < 2; ++i)
        mp_.map_origin_(i) += shift(i) * mp_.resolution_;
    mp_.map_min_boundary_ = mp_.map_origin_;
    mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;
}

void MapCore::raycast(const Eigen::Vector2i &start, const Eigen::Vector2i &end)
{
    int x0 = start(0), y0 = start(1);
    int x1 = end(0),   y1 = end(1);
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (x0 != x1 || y0 != y1) {
        Eigen::Vector2i cell(x0, y0);
        if (isInMap(cell)) {
            int addr = toAddress(cell);
            md_.logodds_buffer_[addr] += (float)mp_.logodds_miss_;
            if (md_.logodds_buffer_[addr] < (float)mp_.logodds_min_)
                md_.logodds_buffer_[addr] = (float)mp_.logodds_min_;
        }

        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

void MapCore::thresholdLogodds(double now)
{
    // 占据 = logodds 超阈值 且 最近被命中过 (occ_timeout_ <= 0 时关闭龄期门控)。
    // 只影响局部更新窗口内的格子；窗口外维持原状，远处未复测的真实墙体不会凭空过期。
    const bool use_timeout = mp_.occ_timeout_ > 0.0;
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
        for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
        {
            int addr = toAddress(x, y);
            bool occ = md_.logodds_buffer_[addr] > (float)mp_.logodds_thresh_;
            if (occ && use_timeout &&
                now - (double)md_.last_hit_time_[addr] > mp_.occ_timeout_)
                occ = false;
            md_.occupancy_buffer_inflate_[addr] = occ ? 1 : 0;
        }
}

template <typename F_get_val, typename F_set_val>
void MapCore::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim)
{
    std::vector<int> v(mp_.map_voxel_num_(dim));
    std::vector<double> z(mp_.map_voxel_num_(dim) + 1);

    int k = start;
    v[start] = start;
    z[start] = -std::numeric_limits<double>::max();
    z[start + 1] = std::numeric_limits<double>::max();

    for (int q = start + 1; q <= end; q++)
    {
        k++;
        double s;

        do
        {
            k--;
            s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
        } while (s <= z[k]);

        k++;

        v[k] = q;
        z[k] = s;
        z[k + 1] = std::numeric_limits<double>::max();
    }

    k = start;

    for (int q = start; q <= end; q++)
    {
        while (z[k + 1] < q)
            k++;
        double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
        f_set_val(q, val);
    }
}

void MapCore::updateESDF2d()
{
    Eigen::Vector2i min_esdf = md_.local_bound_min_;
    Eigen::Vector2i max_esdf = md_.local_bound_max_;

    /* compute positive DT */
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
    {
        fillESDF(
            [&](int y) {
                return md_.is_occupancy(toAddress(x, y)) == 1 ? 0 : std::numeric_limits<double>::max();
            },
            [&](int y, double val) { md_.tmp_buffer1_[toAddress(x, y)] = val; },
            min_esdf[1], max_esdf[1], 1);
    }
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
    {
        fillESDF(
            [&](int x) { return md_.tmp_buffer1_[toAddress(x, y)]; },
            [&](int x, double val) { md_.distance_buffer_[toAddress(x, y)] = sqrt(val) * mp_.resolution_; },
            min_esdf[0], max_esdf[0], 0);
    }

    /* compute negative distance */
    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
        for (int y = min_esdf(1); y <= max_esdf(1); ++y)
        {
            int idx = toAddress(x, y);
            if (md_.is_occupancy(idx) == 0)
                md_.occupancy_buffer_neg[idx] = 1;
            else
                md_.occupancy_buffer_neg[idx] = 0;
        }

    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
    {
        fillESDF(
            [&](int y) {
                return md_.occupancy_buffer_neg[toAddress(x, y)] == 1 ? 0 : std::numeric_limits<double>::max();
            },
            [&](int y, double val) { md_.tmp_buffer1_[toAddress(x, y)] = val; },
            min_esdf[1], max_esdf[1], 1);
    }
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
    {
        fillESDF(
            [&](int x) { return md_.tmp_buffer1_[toAddress(x, y)]; },
            [&](int x, double val) { md_.distance_buffer_neg_[toAddress(x, y)] = sqrt(val) * mp_.resolution_; },
            min_esdf[0], max_esdf[0], 0);
    }

    /* combine pos and neg DT */
    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
        for (int y = min_esdf(1); y <= max_esdf(1); ++y)
        {
            int idx = toAddress(x, y);
            md_.distance_buffer_all_[idx] = md_.distance_buffer_[idx];

            if (md_.distance_buffer_neg_[idx] > 0.0)
                md_.distance_buffer_all_[idx] += (-md_.distance_buffer_neg_[idx] + mp_.resolution_);
        }
}
