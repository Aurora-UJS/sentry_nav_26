/**
 * TrajectoryTracker 窄道/坡道对齐模式测试
 *
 * 覆盖: 进/出迟滞、对齐后恒速、未对齐先转向、车尾对齐等效、
 *       坡道倾角门控、无轨迹自转门控、纯追踪横向纠偏。
 */
#include <gtest/gtest.h>
#include <sentry_local_planner/trajectory_tracker.hpp>

using sentry_planner::MincoTrajectory;
using sentry_planner::TrajectoryTracker;

namespace
{

/** 恒定 ESDF 距离环境（模拟均匀宽度走廊 / 开阔地） */
class ConstEnv : public sentry_nav::EnvironmentInterface
{
public:
    double dist = 100.0;

    double getDistance(const Eigen::Vector2d &) override { return dist; }
    int getInflateOccupancy(const Eigen::Vector2d &) override { return 0; }
    bool isInMap(const Eigen::Vector2d &) override { return true; }
    void evaluateEDTWithGrad(const Eigen::Vector2d &, double,
                             double &d, Eigen::Vector2d &grad) override
    {
        d = dist;
        grad = Eigen::Vector2d(0.0, 1.0);
    }
    double evaluateCoarseEDT(const Eigen::Vector2d &, double) override { return dist; }
    bool hasDynamicObjects() const override { return false; }
    void getMapRegion(Eigen::Vector2d &ori, Eigen::Vector2d &size) override
    {
        ori = Eigen::Vector2d(-50, -50);
        size = Eigen::Vector2d(100, 100);
    }
    double getResolution() override { return 0.05; }
};

/** (0,0) → (len,0) 直线轨迹（纯五次多项式，无优化器） */
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

TrajectoryTracker makeTracker(ConstEnv &env)
{
    TrajectoryTracker::Config cfg;
    cfg.spin_rate = 3.0;
    cfg.corridor_enter_dist = 0.55;
    cfg.corridor_exit_dist = 0.80;
    cfg.corridor_preview_time = 1.2;
    cfg.corridor_speed = 0.8;
    cfg.corridor_align_speed = 0.2;
    cfg.corridor_align_tol = 0.35;
    cfg.corridor_lookahead = 0.6;
    cfg.slope_enter_rad = 0.122;  // ~7°
    cfg.slope_exit_rad = 0.070;   // ~4°
    cfg.yaw_align_kp = 3.0;
    cfg.max_align_wz = 2.5;
    cfg.world_frame_cmd = true;  // linear 保持 odom 系，便于断言
    TrajectoryTracker tracker;
    tracker.init(cfg, &env);
    return tracker;
}

constexpr double kSpin = 3.0;
const Eigen::Vector2d kZeroV = Eigen::Vector2d::Zero();

} // namespace

TEST(TrackerMode, NormalModeSpinsInOpenSpace)
{
    ConstEnv env;  // dist=100 开阔
    auto tracker = makeTracker(env);
    auto traj = makeStraightTraj(3.0);

    auto cmd = tracker.compute({0.5, 0.0}, kZeroV, 0.0, 0.0, traj, 0.5, true);
    EXPECT_FALSE(tracker.inAlignMode());
    EXPECT_DOUBLE_EQ(cmd.angular.z, kSpin);
}

TEST(TrackerMode, NarrowEnterExitHysteresis)
{
    ConstEnv env;
    auto tracker = makeTracker(env);
    auto traj = makeStraightTraj(3.0);
    auto step = [&] { tracker.compute({0.5, 0.0}, kZeroV, 0.0, 0.0, traj, 0.5, true); };

    env.dist = 0.5;  // < enter 0.55 → 进入
    step();
    EXPECT_TRUE(tracker.inNarrowMode());

    env.dist = 0.7;  // enter < 0.7 < exit → 保持（迟滞）
    step();
    EXPECT_TRUE(tracker.inNarrowMode());

    env.dist = 0.9;  // > exit 0.80 → 退出
    step();
    EXPECT_FALSE(tracker.inNarrowMode());

    env.dist = 0.7;  // 退出后回到迟滞带 → 不重入
    step();
    EXPECT_FALSE(tracker.inNarrowMode());

    env.dist = 0.5;
    step();
    EXPECT_TRUE(tracker.inNarrowMode());
}

TEST(TrackerMode, ConstantSpeedWhenAligned)
{
    ConstEnv env;
    env.dist = 0.4;
    auto tracker = makeTracker(env);
    auto traj = makeStraightTraj(3.0);

    // 航向已对齐 +x 轨迹方向 → 恒速直穿，不自转
    auto cmd = tracker.compute({0.5, 0.0}, kZeroV, 0.0, 0.0, traj, 0.5, true);
    EXPECT_TRUE(tracker.inNarrowMode());
    double speed = std::hypot(cmd.linear.x, cmd.linear.y);
    EXPECT_NEAR(speed, 0.8, 1e-6);
    EXPECT_NEAR(cmd.linear.y, 0.0, 1e-6);
    EXPECT_NEAR(cmd.angular.z, 0.0, 0.1);
}

TEST(TrackerMode, AlignFirstWhenMisaligned)
{
    ConstEnv env;
    env.dist = 0.4;
    auto tracker = makeTracker(env);
    auto traj = makeStraightTraj(3.0);

    // 航向偏 0.8 rad (~46°) → 爬行速度 + 向对齐方向转（斜着冲窄口=加宽自己）
    auto cmd = tracker.compute({0.5, 0.0}, kZeroV, 0.8, 0.0, traj, 0.5, true);
    double speed = std::hypot(cmd.linear.x, cmd.linear.y);
    EXPECT_NEAR(speed, 0.2, 1e-6);
    EXPECT_LT(cmd.angular.z, -1.0);  // kp*(-0.8) clamp 后 -2.4
    EXPECT_GE(cmd.angular.z, -2.5);  // 不超上限
}

TEST(TrackerMode, TailFirstIsAligned)
{
    ConstEnv env;
    env.dist = 0.4;
    auto tracker = makeTracker(env);
    auto traj = makeStraightTraj(3.0);

    // 车尾朝轨迹方向 (yaw=π)：误差折到 0，车尾对齐等效可行，不掉头
    auto cmd = tracker.compute({0.5, 0.0}, kZeroV, M_PI, 0.0, traj, 0.5, true);
    double speed = std::hypot(cmd.linear.x, cmd.linear.y);
    EXPECT_NEAR(speed, 0.8, 1e-6);
    EXPECT_NEAR(cmd.angular.z, 0.0, 0.1);
}

TEST(TrackerMode, SlopeEnterExitHysteresis)
{
    ConstEnv env;  // 开阔地，只有倾角触发
    auto tracker = makeTracker(env);
    auto traj = makeStraightTraj(3.0);
    auto stepTilt = [&](double tilt) {
        return tracker.compute({0.5, 0.0}, kZeroV, 0.0, tilt, traj, 0.5, true);
    };

    auto cmd = stepTilt(0.15);  // > enter 0.122 (~8.6°) → 坡道模式
    EXPECT_TRUE(tracker.inSlopeMode());
    EXPECT_LT(std::fabs(cmd.angular.z), 1.0);  // 不再 3.0 自转

    stepTilt(0.10);  // exit < 0.10 < enter → 保持
    EXPECT_TRUE(tracker.inSlopeMode());

    cmd = stepTilt(0.05);  // < exit 0.070 → 退出，恢复自转
    EXPECT_FALSE(tracker.inSlopeMode());
    EXPECT_DOUBLE_EQ(cmd.angular.z, kSpin);
}

TEST(TrackerMode, NoTrajSpinGated)
{
    ConstEnv env;
    auto tracker = makeTracker(env);
    MincoTrajectory empty;

    // 开阔 + 平地 → 搜索自转
    auto cmd = tracker.compute({0.0, 0.0}, kZeroV, 0.0, 0.0, empty, 0.0, false);
    EXPECT_DOUBLE_EQ(cmd.angular.z, kSpin);

    // 贴墙（esdf < exit_dist）→ 禁止自转（扫墙）
    env.dist = 0.5;
    cmd = tracker.compute({0.0, 0.0}, kZeroV, 0.0, 0.0, empty, 0.0, false);
    EXPECT_DOUBLE_EQ(cmd.angular.z, 0.0);

    // 坡上（倾角超限）→ 禁止自转（侧翻风险）
    env.dist = 100.0;
    cmd = tracker.compute({0.0, 0.0}, kZeroV, 0.0, 0.15, empty, 0.0, false);
    EXPECT_DOUBLE_EQ(cmd.angular.z, 0.0);
}

TEST(TrackerMode, PurePursuitSteersBackToPath)
{
    ConstEnv env;
    env.dist = 0.4;
    auto tracker = makeTracker(env);
    auto traj = makeStraightTraj(3.0);

    // 机器人偏在轨迹 (y=0) 上方 0.3m → 指令应含向 -y 的纠偏分量
    auto cmd = tracker.compute({0.5, 0.3}, kZeroV, 0.0, 0.0, traj, 0.5, true);
    EXPECT_LT(cmd.linear.y, -0.05);
    EXPECT_GT(cmd.linear.x, 0.0);
}
