/**
 * MapCore 占据超时 (occ_timeout_) 单元测试
 *
 * 覆盖: 超时过期、再命中续期、timeout<=0 关闭、滑窗清理重置时间戳。
 */
#include <gtest/gtest.h>
#include <plan_env/map_core.hpp>

namespace
{

MapCore makeCore(double occ_timeout)
{
    MappingParameters mp;
    mp.map_size_ = Eigen::Vector2d(2.0, 2.0);
    mp.resolution_ = 0.1;
    mp.resolution_inv_ = 1.0 / mp.resolution_;
    mp.map_origin_ = Eigen::Vector2d(-1.0, -1.0);
    mp.map_voxel_num_ = Eigen::Vector2i(20, 20);
    mp.map_min_boundary_ = mp.map_origin_;
    mp.map_max_boundary_ = mp.map_origin_ + mp.map_size_;
    mp.logodds_hit_ = 0.85;
    mp.logodds_miss_ = -0.4;
    mp.logodds_max_ = 3.5;
    mp.logodds_min_ = -2.0;
    mp.logodds_thresh_ = 0.0;
    mp.occ_timeout_ = occ_timeout;

    MapCore core;
    core.initBuffers(mp, mp.map_voxel_num_(0) * mp.map_voxel_num_(1));
    core.md_.local_bound_min_ = Eigen::Vector2i(0, 0);
    core.md_.local_bound_max_ = Eigen::Vector2i(19, 19);
    return core;
}

void hitCell(MapCore &core, const Eigen::Vector2i &idx, double t)
{
    int addr = core.toAddress(idx);
    core.md_.logodds_buffer_[addr] += (float)core.mp_.logodds_hit_;
    if (core.md_.logodds_buffer_[addr] > (float)core.mp_.logodds_max_)
        core.md_.logodds_buffer_[addr] = (float)core.mp_.logodds_max_;
    core.md_.last_hit_time_[addr] = (float)t;
}

int occAt(MapCore &core, const Eigen::Vector2i &idx)
{
    return core.md_.occupancy_buffer_inflate_[core.toAddress(idx)];
}

TEST(OccTimeout, ExpiresAfterTimeout)
{
    auto core = makeCore(3.5);
    Eigen::Vector2i cell(10, 10);
    hitCell(core, cell, 100.0);

    core.thresholdLogodds(100.1);
    EXPECT_EQ(occAt(core, cell), 1) << "刚命中应为占据";

    core.thresholdLogodds(103.0);
    EXPECT_EQ(occAt(core, cell), 1) << "未超时应保持占据";

    core.thresholdLogodds(103.6);
    EXPECT_EQ(occAt(core, cell), 0) << "超时 3.5s 后应过期为空闲";
}

TEST(OccTimeout, RehitRefreshes)
{
    auto core = makeCore(3.5);
    Eigen::Vector2i cell(5, 5);
    hitCell(core, cell, 100.0);
    hitCell(core, cell, 103.0); // 续期

    core.thresholdLogodds(104.0);
    EXPECT_EQ(occAt(core, cell), 1) << "再命中续期后不应过期";

    core.thresholdLogodds(106.6);
    EXPECT_EQ(occAt(core, cell), 0) << "距最后命中超 3.5s 后过期";
}

TEST(OccTimeout, DisabledKeepsOldBehavior)
{
    auto core = makeCore(0.0);
    Eigen::Vector2i cell(3, 7);
    hitCell(core, cell, 100.0);

    core.thresholdLogodds(1e6);
    EXPECT_EQ(occAt(core, cell), 1) << "timeout<=0 时永不过期 (旧行为)";
}

TEST(OccTimeout, NeverHitStaysFree)
{
    auto core = makeCore(3.5);
    Eigen::Vector2i cell(2, 2);
    core.thresholdLogodds(50.0);
    EXPECT_EQ(occAt(core, cell), 0);
}

TEST(OccTimeout, SlideResetsStamp)
{
    auto core = makeCore(3.5);
    Eigen::Vector2i cell(10, 10);
    hitCell(core, cell, 100.0);
    int addr = core.toAddress(cell);

    // 滑窗清除整列后时间戳应回到 kNeverHit
    core.clearRingSlice(0, 10, 11);
    EXPECT_FLOAT_EQ(core.md_.last_hit_time_[addr], MappingData::kNeverHit);
    EXPECT_FLOAT_EQ(core.md_.logodds_buffer_[addr], 0.0f);
}

} // namespace
