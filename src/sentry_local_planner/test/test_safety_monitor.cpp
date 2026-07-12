/**
 * checkTrajectorySafety 测试：mock ESDF 环境 + 直线 MINCO 轨迹
 * 分级语义：warn (进降速) / hard (真穿障，允许刹停) / 迟滞退出
 */
#include <gtest/gtest.h>
#include <sentry_local_planner/safety_monitor.hpp>

using sentry_planner::checkTrajectorySafety;
using sentry_planner::MincoTrajectory;

namespace
{

/**
 * 半平面墙: x >= wall_x 为障碍，ESDF = wall_x - x（墙内为 0）
 */
class MockEnv : public sentry_nav::EnvironmentInterface
{
public:
    double wall_x = 1e9;
    bool in_map = true;

    double getDistance(const Eigen::Vector2d &pos) override
    {
        return std::max(0.0, wall_x - pos.x());
    }
    int getInflateOccupancy(const Eigen::Vector2d &pos) override
    {
        return pos.x() >= wall_x ? 1 : 0;
    }
    bool isInMap(const Eigen::Vector2d &) override { return in_map; }
    void evaluateEDTWithGrad(const Eigen::Vector2d &pos, double,
                             double &dist, Eigen::Vector2d &grad) override
    {
        dist = getDistance(pos);
        grad = Eigen::Vector2d(-1.0, 0.0);
    }
    double evaluateCoarseEDT(const Eigen::Vector2d &pos, double) override
    {
        return getDistance(pos);
    }
    bool hasDynamicObjects() const override { return false; }
    void getMapRegion(Eigen::Vector2d &ori, Eigen::Vector2d &size) override
    {
        ori = Eigen::Vector2d(-50, -50);
        size = Eigen::Vector2d(100, 100);
    }
    double getResolution() override { return 0.05; }
};

/** (0,0) → (len,0) 直线轨迹，无优化器（纯五次多项式重建） */
MincoTrajectory makeStraightTraj(double len, double max_vel = 1.0)
{
    MincoTrajectory traj;
    std::vector<Eigen::Vector2d> wps = {
        {0.0, 0.0}, {len / 2.0, 0.0}, {len, 0.0}};
    traj.setup(wps, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
               Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
               max_vel, 1.0, -1.0);
    return traj;
}

constexpr double kDt = 0.03;
constexpr double kWarn = 0.20;
constexpr double kHard = 0.15;
constexpr double kIgnoreR = 0.35;
const Eigen::Vector2d kFarAway(-100.0, -100.0);  // 默认当前位置远离轨迹，豁免不生效

} // namespace

TEST(SafetyMonitor, FreeSpaceIsSafe)
{
    MockEnv env;  // 墙在无穷远
    auto traj = makeStraightTraj(3.0);
    auto res = checkTrajectorySafety(env, traj, 0.0, traj.getDuration(), kDt, kWarn, kHard, kFarAway, kIgnoreR);
    EXPECT_FALSE(res.warn());
    EXPECT_FALSE(res.hard());
}

TEST(SafetyMonitor, WallOnPathTriggersWarnThenHard)
{
    MockEnv env;
    env.wall_x = 2.0;  // 轨迹终点 3.0 > 墙，必然穿墙
    auto traj = makeStraightTraj(3.0);
    auto res = checkTrajectorySafety(env, traj, 0.0, traj.getDuration(), kDt, kWarn, kHard, kFarAway, kIgnoreR);
    ASSERT_TRUE(res.warn());
    ASSERT_TRUE(res.hard());
    // 逼近过程先触 warn 再触 hard
    EXPECT_LE(res.first_warn_time, res.first_hard_time);

    // 各自首触时刻应落在对应阈值带内
    EXPECT_LT(env.getDistance(traj.getPosition(res.first_warn_time)), kWarn);
    EXPECT_LT(env.getDistance(traj.getPosition(res.first_hard_time)), kHard);
    // warn 首触之前一步仍在阈值之上（确为最早时刻）
    if (res.first_warn_time >= kDt)
        EXPECT_GE(env.getDistance(traj.getPosition(res.first_warn_time - kDt)), kWarn);
}

// 规划器有意的贴墙轨迹（间隙略高于 warn）不应报警 —— 防振荡的核心语义
TEST(SafetyMonitor, IntentionalWallHuggingIsNotFlagged)
{
    MockEnv env;
    env.wall_x = 3.28;  // 轨迹终点 x=3.0，最小间隙 0.28m（MINCO 贴墙典型值）
    auto traj = makeStraightTraj(3.0);
    auto res = checkTrajectorySafety(env, traj, 0.0, traj.getDuration(), kDt, kWarn, kHard, kFarAway, kIgnoreR);
    EXPECT_FALSE(res.warn());
    EXPECT_FALSE(res.hard());
    EXPECT_NEAR(res.min_distance, 0.28, 0.05);
}

// 间隙落在 warn 与 hard 之间：只警告，不判穿障
TEST(SafetyMonitor, MarginalClearanceWarnsWithoutHard)
{
    MockEnv env;
    env.wall_x = 3.17;  // 最小间隙 0.17m ∈ (hard 0.15, warn 0.20)
    auto traj = makeStraightTraj(3.0);
    auto res = checkTrajectorySafety(env, traj, 0.0, traj.getDuration(), kDt, kWarn, kHard, kFarAway, kIgnoreR);
    EXPECT_TRUE(res.warn());
    EXPECT_FALSE(res.hard());
}

TEST(SafetyMonitor, WindowBeforeWallIsSafe)
{
    MockEnv env;
    env.wall_x = 2.5;
    auto traj = makeStraightTraj(3.0);
    // 只检查轨迹前段（远离墙），应判安全 —— 前瞻窗口语义
    auto res_front = checkTrajectorySafety(env, traj, 0.0, traj.getDuration() * 0.3, kDt, kWarn, kHard, kFarAway, kIgnoreR);
    EXPECT_FALSE(res_front.warn());
    // 全窗口则触发
    auto res_full = checkTrajectorySafety(env, traj, 0.0, traj.getDuration(), kDt, kWarn, kHard, kFarAway, kIgnoreR);
    EXPECT_TRUE(res_full.hard());
}

TEST(SafetyMonitor, OutOfMapIsHardUnsafe)
{
    MockEnv env;
    env.in_map = false;  // 全部采样点出地图
    auto traj = makeStraightTraj(3.0);
    auto res = checkTrajectorySafety(env, traj, 0.0, traj.getDuration(), kDt, kWarn, kHard, kFarAway, kIgnoreR);
    EXPECT_TRUE(res.hard());
    EXPECT_DOUBLE_EQ(res.first_hard_time, 0.0);
}

TEST(SafetyMonitor, DegenerateInputsAreSafeNoop)
{
    MockEnv env;
    env.wall_x = 0.5;
    MincoTrajectory empty;
    EXPECT_FALSE(checkTrajectorySafety(env, empty, 0.0, 1.0, kDt, kWarn, kHard, kFarAway, kIgnoreR).warn());

    auto traj = makeStraightTraj(3.0);
    EXPECT_FALSE(checkTrajectorySafety(env, traj, 1.0, 1.0, kDt, kWarn, kHard, kFarAway, kIgnoreR).warn());   // 空窗口
    EXPECT_FALSE(checkTrajectorySafety(env, traj, 0.0, 1.0, 0.0, kWarn, kHard, kFarAway, kIgnoreR).warn());   // dt=0
}

// min_distance 全窗口统计（供退出迟滞判断），不因命中阈值提前截断
TEST(SafetyMonitor, MinDistanceIsTracked)
{
    MockEnv env;
    env.wall_x = 5.0;  // 安全但可量化：终点 x=3 处 ESDF = 2.0
    auto traj = makeStraightTraj(3.0);
    auto res = checkTrajectorySafety(env, traj, 0.0, traj.getDuration(), kDt, kWarn, kHard, kFarAway, kIgnoreR);
    EXPECT_FALSE(res.warn());
    EXPECT_NEAR(res.min_distance, 2.0, 0.1);

    env.wall_x = 2.0;  // 穿墙情形下 min_distance 也应扫到 0（墙内），不在 warn 处截断
    auto res2 = checkTrajectorySafety(env, traj, 0.0, traj.getDuration(), kDt, kWarn, kHard, kFarAway, kIgnoreR);
    EXPECT_TRUE(res2.hard());
    EXPECT_NEAR(res2.min_distance, 0.0, 1e-9);
}

// 脱困豁免：距机器人当前位置过近的采样点不参与判定，
// 否则贴墙被困时规划器的脱困轨迹会被同拍打回 BRAKE 形成死循环
TEST(SafetyMonitor, SelfIgnoreRadiusExemptsEscapeManeuver)
{
    MockEnv env;
    env.wall_x = 0.30;  // 轨迹 (0,0)->(3,0) 直插墙内：x>0.15 起 esdf<0.15 硬不安全
    auto traj = makeStraightTraj(3.0);
    Eigen::Vector2d at_start(0.0, 0.0);

    // 当前位置远离轨迹（豁免不生效）：首个硬不安全点在 x≈0.15
    auto res_no_exempt = checkTrajectorySafety(
        env, traj, 0.0, traj.getDuration(), kDt, kWarn, kHard, kFarAway, kIgnoreR);
    ASSERT_TRUE(res_no_exempt.hard());

    // 当前位置在轨迹起点：0.35m 豁免圈内的采样点跳过，
    // 但圈外 (x>0.35, esdf=0) 依然是硬不安全 —— 豁免不削弱远处的前瞻
    auto res_at_start = checkTrajectorySafety(
        env, traj, 0.0, traj.getDuration(), kDt, kWarn, kHard, at_start, kIgnoreR);
    EXPECT_TRUE(res_at_start.hard());
    EXPECT_GE((traj.getPosition(res_at_start.first_hard_time) - at_start).norm(), kIgnoreR);
    EXPECT_GT(res_at_start.first_hard_time, res_no_exempt.first_hard_time);

    // 豁免圈覆盖整条轨迹时判定完全解除（极端参数下的行为边界）
    auto res_big_ignore = checkTrajectorySafety(
        env, traj, 0.0, traj.getDuration(), kDt, kWarn, kHard, at_start, 100.0);
    EXPECT_FALSE(res_big_ignore.hard());
    EXPECT_FALSE(res_big_ignore.warn());
}
