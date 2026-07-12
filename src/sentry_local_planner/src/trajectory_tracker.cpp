#include <sentry_local_planner/trajectory_tracker.hpp>
#include <algorithm>
#include <cmath>

namespace sentry_planner
{

void TrajectoryTracker::init(const Config &cfg, sentry_nav::EnvironmentInterface *env)
{
    cfg_ = cfg;
    env_ = env;

    MPCController::Config mpc_cfg;
    mpc_cfg.horizon = cfg.mpc_horizon;
    mpc_cfg.dt = cfg.ctrl_dt;
    mpc_cfg.q_pos = cfg.mpc_q_pos;
    mpc_cfg.q_vel = cfg.mpc_q_vel;
    mpc_cfg.r_acc = cfg.mpc_r_acc;
    mpc_cfg.max_vel = cfg.max_vel;
    mpc_cfg.max_acc = cfg.max_acc;
    mpc_.setConfig(mpc_cfg);
}

geometry_msgs::msg::Twist TrajectoryTracker::compute(
    const Eigen::Vector2d &pos, const Eigen::Vector2d &vel, double yaw, double tilt,
    const MincoTrajectory &traj, double elapsed, bool has_traj)
{
    geometry_msgs::msg::Twist cmd;

    if (!has_traj)
    {
        // 无轨迹搜索自转 —— 但坡上自转有侧翻风险，贴墙自转会扫墙，都禁止
        slope_on_ = slope_on_ ? (tilt > cfg_.slope_exit_rad) : (tilt > cfg_.slope_enter_rad);
        bool near_wall = env_ && env_->isInMap(pos) &&
                         env_->getDistance(pos) < cfg_.corridor_exit_dist;
        cmd.angular.z = (slope_on_ || near_wall) ? 0.0 : cfg_.spin_rate;
        return cmd;
    }

    updateMode(pos, tilt, traj, elapsed);

    if (narrow_on_ || slope_on_)
        return computeAlignCmd(pos, yaw, traj, elapsed);

    // --- 常规模式: MPC + 自转 ---
    auto ref_func = [&traj](double t) -> Eigen::Vector4d {
        Eigen::Vector4d ref;
        ref.head(2) = traj.getPosition(t);
        ref.tail(2) = traj.getVelocity(t);
        return ref;
    };

    Eigen::Vector2d acc_cmd = mpc_.compute(pos, vel, ref_func, elapsed);
    Eigen::Vector2d vel_ref = traj.getVelocity(elapsed);
    double dt_lookahead = cfg_.ctrl_dt * 3.0;
    Eigen::Vector2d vel_cmd_odom = vel_ref + acc_cmd * dt_lookahead;

    // 限速
    if (vel_cmd_odom.norm() > cfg_.max_vel)
        vel_cmd_odom = vel_cmd_odom.normalized() * cfg_.max_vel;

    if (cfg_.world_frame_cmd)
    {
        // 世界系输出：机体系旋转下沉到 chassis_cmd_node / 电控侧
        cmd.linear.x = vel_cmd_odom(0);
        cmd.linear.y = vel_cmd_odom(1);
    }
    else
    {
        // Odom → body frame
        double cy = cos(-yaw), sy = sin(-yaw);
        cmd.linear.x = cy * vel_cmd_odom(0) - sy * vel_cmd_odom(1);
        cmd.linear.y = sy * vel_cmd_odom(0) + cy * vel_cmd_odom(1);
    }
    cmd.angular.z = cfg_.spin_rate;

    return cmd;
}

void TrajectoryTracker::updateMode(const Eigen::Vector2d &pos, double tilt,
                                   const MincoTrajectory &traj, double elapsed)
{
    slope_on_ = slope_on_ ? (tilt > cfg_.slope_exit_rad) : (tilt > cfg_.slope_enter_rad);

    if (!env_)
    {
        narrow_on_ = false;
        return;
    }

    // 当前位置 + 轨迹前方 preview_time 采样的最小 ESDF（出地图按 0 处理）
    double dmin = env_->isInMap(pos) ? env_->getDistance(pos) : 0.0;
    double t_end = std::min(elapsed + cfg_.corridor_preview_time, traj.getDuration());
    for (double t = std::max(elapsed, 0.0); t <= t_end; t += 0.1)
    {
        Eigen::Vector2d p = traj.getPosition(t);
        double d = env_->isInMap(p) ? env_->getDistance(p) : 0.0;
        dmin = std::min(dmin, d);
    }
    narrow_on_ = narrow_on_ ? (dmin < cfg_.corridor_exit_dist)
                            : (dmin < cfg_.corridor_enter_dist);
}

geometry_msgs::msg::Twist TrajectoryTracker::computeAlignCmd(
    const Eigen::Vector2d &pos, double yaw,
    const MincoTrajectory &traj, double elapsed)
{
    geometry_msgs::msg::Twist cmd;
    const double dur = traj.getDuration();
    const double kStep = 0.05;

    // 最近点: 在 elapsed 邻域内搜索（恒速与时间参考脱钩后 elapsed 会漂，
    // 但重规划的偏差触发 (0.5m) 保证漂移有界，邻域覆盖它）
    double t0 = std::clamp(elapsed - 0.5, 0.0, dur);
    double t1 = std::min(elapsed + 2.0, dur);
    double t_near = t0, best = 1e18;
    for (double t = t0; t <= t1; t += kStep)
    {
        double d2 = (traj.getPosition(t) - pos).squaredNorm();
        if (d2 < best) { best = d2; t_near = t; }
    }

    // 前瞻点: 从最近点沿轨迹向前累计弧长 corridor_lookahead
    Eigen::Vector2d prev = traj.getPosition(t_near), lk = prev;
    double s = 0.0;
    for (double t = t_near + kStep; t <= dur && s < cfg_.corridor_lookahead; t += kStep)
    {
        Eigen::Vector2d p = traj.getPosition(t);
        s += (p - prev).norm();
        prev = p;
        lk = p;
    }

    // 前瞻点向脊线推移后按对齐策略输出
    nudgeToRidge(lk);
    return alignCmdToward(pos, yaw, lk);
}

geometry_msgs::msg::Twist TrajectoryTracker::computeCrawl(
    const Eigen::Vector2d &pos, double yaw, Eigen::Vector2d target)
{
    nudgeToRidge(target);
    return alignCmdToward(pos, yaw, target);
}

void TrajectoryTracker::nudgeToRidge(Eigen::Vector2d &p) const
{
    // 目标点向 ESDF 脊线（缝中心线）推移: MINCO 的软间隙代价在 esdf<dist0
    // 后已无区分度，缝内轨迹可能偏离中心；直穿时跟最大间隙线更稳
    if (!env_ || !env_->isInMap(p))
        return;
    for (int i = 0; i < 3; ++i)
    {
        double d;
        Eigen::Vector2d g;
        env_->evaluateEDTWithGrad(p, -1.0, d, g);
        if (d >= cfg_.corridor_enter_dist || g.norm() < 1e-3)
            break;
        p += g.normalized() * 0.05;
    }
}

geometry_msgs::msg::Twist TrajectoryTracker::alignCmdToward(
    const Eigen::Vector2d &pos, double yaw, const Eigen::Vector2d &target) const
{
    geometry_msgs::msg::Twist cmd;
    Eigen::Vector2d dir = target - pos;
    if (dir.norm() < 0.05)
        return cmd;  // 已贴近目标: 停车
    dir.normalize();

    // 航向对齐: 折到 [-π/2, π/2] —— 车头或车尾对齐皆可（前后对称），不侧行
    double desired_yaw = atan2(dir(1), dir(0));
    double yaw_err = desired_yaw - yaw;
    while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;
    if (yaw_err > M_PI / 2.0) yaw_err -= M_PI;
    else if (yaw_err < -M_PI / 2.0) yaw_err += M_PI;

    cmd.angular.z = std::clamp(cfg_.yaw_align_kp * yaw_err,
                               -cfg_.max_align_wz, cfg_.max_align_wz);

    // 对齐后恒速直穿；未对齐先爬行转向（斜着冲窄口等于加宽自己）
    double speed = std::fabs(yaw_err) < cfg_.corridor_align_tol
                       ? cfg_.corridor_speed
                       : cfg_.corridor_align_speed;
    Eigen::Vector2d v = dir * speed;

    if (cfg_.world_frame_cmd)
    {
        cmd.linear.x = v(0);
        cmd.linear.y = v(1);
    }
    else
    {
        double cy = cos(-yaw), sy = sin(-yaw);
        cmd.linear.x = cy * v(0) - sy * v(1);
        cmd.linear.y = sy * v(0) + cy * v(1);
    }
    return cmd;
}

} // namespace sentry_planner
