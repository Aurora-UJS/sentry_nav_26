/**
 * KinodynamicAstar 单元测试: 薄障碍语义 + near_end 不静默放弃 + 时间预算
 *
 * 背景 (台沿死锁取证): 旧实现两处缺陷——
 *   1) near_end (索引切比雪夫 ≤1m) 起点 shot 穿墙失败即静默 NO_PATH (零扩展零日志);
 *   2) 5 点二值 footprint 采样从 ≤0.2m 薄障碍带的采样点间穿过。
 * 本测试用解析 ESDF 环境钉死修复后的行为。
 */
#include <gtest/gtest.h>
#include <chrono>
#include <path_searching/kinodynamic_astar.hpp>

using fast_planner::KinodynamicAstar;

namespace
{

/** 轴对齐矩形障碍集合的解析 ESDF 环境 (障碍内为负) */
class RectEnv : public sentry_nav::EnvironmentInterface
{
public:
    struct Rect { double x1, y1, x2, y2; };
    std::vector<Rect> rects;
    double half_size = 12.0;  // 地图 [-half,half]^2

    double sd(const Rect &r, const Eigen::Vector2d &p) const
    {
        double dx = std::max({r.x1 - p.x(), 0.0, p.x() - r.x2});
        double dy = std::max({r.y1 - p.y(), 0.0, p.y() - r.y2});
        if (dx > 0.0 || dy > 0.0)
            return std::hypot(dx, dy);
        // 内部: 负的最小穿透深度
        return -std::min({p.x() - r.x1, r.x2 - p.x(), p.y() - r.y1, r.y2 - p.y()});
    }
    double getDistance(const Eigen::Vector2d &p) override
    {
        double d = 1e9;
        for (const auto &r : rects)
            d = std::min(d, sd(r, p));
        return d;
    }
    int getInflateOccupancy(const Eigen::Vector2d &p) override
    {
        return getDistance(p) <= 0.0 ? 1 : 0;
    }
    bool isInMap(const Eigen::Vector2d &p) override
    {
        return std::fabs(p.x()) < half_size && std::fabs(p.y()) < half_size;
    }
    void evaluateEDTWithGrad(const Eigen::Vector2d &p, double,
                             double &d, Eigen::Vector2d &grad) override
    {
        d = getDistance(p);
        grad = Eigen::Vector2d::Zero();
    }
    double evaluateCoarseEDT(const Eigen::Vector2d &p, double) override
    {
        return getDistance(p);
    }
    bool hasDynamicObjects() const override { return false; }
    void getMapRegion(Eigen::Vector2d &ori, Eigen::Vector2d &size) override
    {
        ori = Eigen::Vector2d(-half_size, -half_size);
        size = Eigen::Vector2d(2 * half_size, 2 * half_size);
    }
    double getResolution() override { return 0.05; }
};

constexpr double kAccept = 0.28;

class AstarTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        if (!rclcpp::ok())
            rclcpp::init(0, nullptr);
    }

    void makeAstar(RectEnv &env)
    {
        rclcpp::NodeOptions opts;
        opts.parameter_overrides({
            {"search.max_tau", 0.6},
            {"search.init_max_tau", 0.8},
            {"search.max_vel", 3.0},
            {"search.max_acc", 2.0},
            {"search.w_time", 5.0},
            {"search.horizon", 15.0},
            {"search.resolution_astar", 0.05},
            {"search.time_resolution", 0.8},
            {"search.lambda_heu", 5.0},
            {"search.allocate_num", 20000},
            {"search.check_num", 50},
            {"search.robot_radius", 0.3},
            {"search.w_clearance", 20.0},
            {"search.start_ignore_radius", 0.35},
            {"search.clearance_dist", 0.5},
            {"search.accept_clearance", kAccept},
            // -O0 测试构建下单次 init 扩展可达 ~40ms, 预算放宽防假失败
            {"search.max_search_time_ms", 150.0},
            {"search.near_end_min_progress", 0.4},
        });
        static int seq = 0;
        node_ = std::make_shared<rclcpp::Node>("astar_test_" + std::to_string(seq++), opts);
        astar_ = std::make_unique<KinodynamicAstar>();
        astar_->setParam(node_);
        astar_->setEnvironment(&env);
        astar_->init();
        astar_->reset();
    }

    /** 轨迹采样是否有穿入障碍的点 (esdf < margin) */
    bool trajViolates(RectEnv &env, double margin,
                      const Eigen::Vector2d &start, double exempt_r)
    {
        for (const auto &p : astar_->getKinoTraj(0.02))
        {
            if ((p - start).norm() < exempt_r)
                continue;  // 起点豁免圈
            if (env.getDistance(p) < margin)
                return true;
        }
        return false;
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<KinodynamicAstar> astar_;
};

} // namespace

TEST_F(AstarTest, CorridorStraightPass)
{
    RectEnv env;  // 1.1m 走廊 (台沿几何量级), 中线 esdf 0.55
    env.rects = {{-10, 0.55, 10, 0.75}, {-10, -0.75, 10, -0.55}};
    makeAstar(env);

    int res = astar_->search({0, 0}, {0, 0}, {0, 0}, {3, 0}, {0, 0}, true);
    EXPECT_EQ(res, KinodynamicAstar::REACH_END);
    if (res != KinodynamicAstar::NO_PATH)
        EXPECT_FALSE(trajViolates(env, kAccept - 0.02, {0, 0}, 0.0));
}

TEST_F(AstarTest, ThinBandNeverStraddled)
{
    // 0.10m 薄带横贯全图, 目标在带对侧 0.5m (在 near_end 框内)。
    // 旧 5 点 footprint 会从采样点间穿带; 旧 near_end 会第一拍静默 NO_PATH。
    // 新行为: 无论返回什么, 轨迹绝不穿带
    RectEnv env;
    env.rects = {{-11.9, -0.05, 11.9, 0.05}};
    makeAstar(env);

    int res = astar_->search({0, 0.5}, {0, 0}, {0, 0}, {0, -0.5}, {0, 0}, true);
    EXPECT_NE(res, KinodynamicAstar::REACH_END) << "全宽薄带不可能到达对侧";
    if (res != KinodynamicAstar::NO_PATH)
        EXPECT_FALSE(trajViolates(env, 0.0, {0, 0.5}, 0.35));
}

TEST_F(AstarTest, DeadlockGeometryGoesAround)
{
    // 台沿死锁几何复刻: 带 y[-5.69,-5.59] 东端 x=3.70, 起点北侧 (3.36,-5.33),
    // 目标带南 (3.34,-6.20) —— 直线 0.87m < near_end 1.0m 但隔带。
    // 旧实现: 第一次迭代 shot 穿带失败 → 静默 NO_PATH (762 次实证)。
    // 新实现: 继续扩展绕过东端 → 可达
    RectEnv env;
    env.rects = {{-10, -5.69, 3.70, -5.59}};
    makeAstar(env);

    int res = astar_->search({3.36, -5.33}, {0, 0}, {0, 0}, {3.34, -6.20}, {0, 0}, true);
    EXPECT_NE(res, KinodynamicAstar::NO_PATH) << "隔带 near_end 不应再静默放弃";
    if (res != KinodynamicAstar::NO_PATH)
        EXPECT_FALSE(trajViolates(env, 0.0, {3.36, -5.33}, 0.35));
    if (res == KinodynamicAstar::REACH_END)
    {
        auto traj = astar_->getKinoTraj(0.02);
        ASSERT_FALSE(traj.empty());
        EXPECT_LT((traj.back() - Eigen::Vector2d(3.34, -6.20)).norm(), 0.3);
    }
}

TEST_F(AstarTest, UnreachableGoalReturnsWithinBudget)
{
    RectEnv env;  // 目标在实心障碍中心, 不可达
    env.rects = {{4.0, -1.0, 6.0, 1.0}};
    makeAstar(env);

    auto t0 = std::chrono::steady_clock::now();
    int res = astar_->search({0, 0}, {0, 0}, {0, 0}, {5.0, 0}, {0, 0}, true);
    double ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t0).count();
    EXPECT_TRUE(res == KinodynamicAstar::NO_PATH || res == KinodynamicAstar::NEAR_END ||
                res == KinodynamicAstar::REACH_HORIZON);
    EXPECT_LT(ms, 200.0) << "时间预算 15ms 失效 (留 CI 裕量断言 200ms)";
}

TEST_F(AstarTest, StartExemptionAllowsDeparture)
{
    // 起点距带 0.10m (< accept 0.28): 豁免圈内起步合法, 应能驶离到开阔目标
    RectEnv env;
    env.rects = {{-11.9, -0.05, 11.9, 0.05}};
    makeAstar(env);

    int res = astar_->search({0, 0.15}, {0, 0}, {0, 0}, {0, 2.5}, {0, 0}, true);
    EXPECT_EQ(res, KinodynamicAstar::REACH_END);
}

TEST_F(AstarTest, GetSamplesEndVelMatchesTrajectoryTail)
{
    // NEAR_END 部分路径: end_vel 必须是路径末端速度而非起点速度。
    // 起点以 +y 1m/s 远离目标出发, 部分路径末端必然转向目标 (-y);
    // 旧 bug 会把 (0,1) 当末速报出, 与末段位移方向相反
    RectEnv env;
    env.rects = {{-11.9, -0.05, 11.9, 0.05}};
    makeAstar(env);

    int res = astar_->search({0, 1.5}, {0, 1.0}, {0, 0}, {0, -0.5}, {0, 0}, true);
    ASSERT_EQ(res, KinodynamicAstar::NEAR_END);

    double ts = 0.1;
    std::vector<Eigen::Vector2d> pts, derivs;
    astar_->getSamples(ts, pts, derivs);
    ASSERT_GE(pts.size(), 2u);
    ASSERT_GE(derivs.size(), 2u);

    Eigen::Vector2d end_vel = derivs[1];
    Eigen::Vector2d tail_dir = pts.back() - pts[pts.size() - 2];
    if (end_vel.norm() > 1e-3 && tail_dir.norm() > 1e-6)
        EXPECT_GT(end_vel.dot(tail_dir), 0.0)
            << "end_vel 与末段位移方向相反 —— 取到了起点速度";
}
