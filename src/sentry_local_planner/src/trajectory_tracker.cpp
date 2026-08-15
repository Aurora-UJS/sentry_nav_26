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

void TrajectoryTracker::updateConfig(const Config &cfg)
{
    cfg_ = cfg;
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
    Eigen::Vector2d v_odom = Eigen::Vector2d::Zero();  // 世界系线速度 (统一出口)

    if (!has_traj)
    {
        // 无轨迹搜索自转 —— 但坡上自转有侧翻风险，贴墙自转会扫墙，都禁止
        slope_on_ = slope_on_ ? (tilt > cfg_.slope_exit_rad) : (tilt > cfg_.slope_enter_rad);
        bool near_wall = env_ && env_->isInMap(pos) &&
                         env_->getDistance(pos) < cfg_.corridor_exit_dist;
        cmd.angular.z = (slope_on_ || near_wall) ? 0.0 : cfg_.spin_rate;
    }
    else
    {
        updateMode(pos, tilt, traj, elapsed);

        if (narrow_on_ || slope_on_)
        {
            cmd.angular.z = computeAlignCmd(pos, yaw, traj, elapsed, v_odom);
        }
        else
        {
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
            v_odom = vel_ref + acc_cmd * dt_lookahead;

            // 限速
            if (v_odom.norm() > cfg_.max_vel)
                v_odom = v_odom.normalized() * cfg_.max_vel;

            cmd.angular.z = cfg_.spin_rate;
        }
    }

    // 线速度限速（世界系，统一出口）：MPC↔对齐切换时速度阶跃（如 0→0.8 瞬时），
    // 坡上瞬时扭矩需求会让麦轮打滑（实测 cmd 0.78 vs 实际位移 0.09，轮子来回蹭）。
    // 用 max_acc 限斜率平滑过渡。
    Eigen::Vector2d dv = v_odom - last_lin_cmd_;
    double dvn = dv.norm();
    double max_dv = cfg_.max_acc * cfg_.ctrl_dt;
    if (dvn > max_dv)
        v_odom = last_lin_cmd_ + dv * (max_dv / dvn);
    last_lin_cmd_ = v_odom;

    // 角速度限速（统一出口）：自转↔对齐切换时 angular.z 阶跃（3.0 ↔ 对齐输出）会甩头，
    // 用斜率限制平滑过渡。angular 与坐标系无关，可直接作用于输出。
    const double max_dwz = cfg_.wz_slew_rate * cfg_.ctrl_dt;
    cmd.angular.z = std::clamp(cmd.angular.z,
                               last_wz_cmd_ - max_dwz, last_wz_cmd_ + max_dwz);
    last_wz_cmd_ = cmd.angular.z;

    // 帧转换 (统一出口)
    if (cfg_.world_frame_cmd)
    {
        cmd.linear.x = v_odom(0);
        cmd.linear.y = v_odom(1);
    }
    else
    {
        double cy = cos(-yaw), sy = sin(-yaw);
        cmd.linear.x = cy * v_odom(0) - sy * v_odom(1);
        cmd.linear.y = sy * v_odom(0) + cy * v_odom(1);
    }

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

    // 真窄道判定（双侧）：轨迹前方 preview_time 内，任意采样点**两侧**侧向 side_probe
    // 处 ESDF 都低于阈值才算窄道。单侧贴墙/贴柱（开阔地里的障碍）不算 —— 正常 MPC+自转
    // 可以绕，不需要停转对齐；只有两侧都被墙夹住的真窄道/夹口才需要。
    const double thr = narrow_on_ ? cfg_.corridor_exit_dist : cfg_.corridor_enter_dist;
    const double probe = cfg_.corridor_side_probe;
    bool tight = false;
    const double t_end = std::min(elapsed + cfg_.corridor_preview_time, traj.getDuration());
    for (double t = std::max(elapsed, 0.0); t <= t_end && !tight; t += 0.1)
    {
        Eigen::Vector2d p = traj.getPosition(t);
        Eigen::Vector2d v = traj.getVelocity(t);
        double vn = v.norm();
        if (vn < 1e-3)
            continue;
        Eigen::Vector2d n(-v(1) / vn, v(0) / vn);  // 左侧法向
        Eigen::Vector2d pl = p + n * probe, pr = p - n * probe;
        double dl = env_->isInMap(pl) ? env_->getDistance(pl) : 0.0;  // 出地图按 0 处理
        double dr = env_->isInMap(pr) ? env_->getDistance(pr) : 0.0;
        if (dl < thr && dr < thr)
        {
            tight = true;
            break;
        }
    }
    narrow_on_ = tight;
}

double TrajectoryTracker::computeAlignCmd(
    const Eigen::Vector2d &pos, double yaw,
    const MincoTrajectory &traj, double elapsed,
    Eigen::Vector2d &v_odom)
{
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

    // 脊线纠偏只对窄道有意义（缝中心线）。坡上禁用：坡脚的 ESDF 梯度方向随车摆动，
    // 会把前瞻点甩来甩去 → 指令方向 90° 摇摆 → 轮子来回打滑（坡脚卡死实锤）。
    if (!slope_on_)
        nudgeToRidge(lk);

    double wz = 0.0;
    alignCmdToward(pos, yaw, lk, v_odom, wz);
    return wz;
}

geometry_msgs::msg::Twist TrajectoryTracker::computeCrawl(
    const Eigen::Vector2d &pos, double yaw, Eigen::Vector2d target)
{
    nudgeToRidge(target);  // 窄道兜底爬行保留脊线纠偏
    Eigen::Vector2d v_odom;
    double wz = 0.0;
    alignCmdToward(pos, yaw, target, v_odom, wz);

    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = wz;
    if (cfg_.world_frame_cmd)
    {
        cmd.linear.x = v_odom(0);
        cmd.linear.y = v_odom(1);
    }
    else
    {
        double cy = cos(-yaw), sy = sin(-yaw);
        cmd.linear.x = cy * v_odom(0) - sy * v_odom(1);
        cmd.linear.y = sy * v_odom(0) + cy * v_odom(1);
    }
    return cmd;
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

bool TrajectoryTracker::alignCmdToward(
    const Eigen::Vector2d &pos, double yaw, const Eigen::Vector2d &target,
    Eigen::Vector2d &v_odom, double &wz) const
{
    Eigen::Vector2d dir = target - pos;
    if (dir.norm() < 0.05)
    {
        v_odom.setZero();
        wz = 0.0;
        return false;  // 已贴近目标: 停车
    }
    dir.normalize();

    // 航向对齐: 折到 [-π/2, π/2] —— 车头或车尾对齐皆可（前后对称），不侧行
    double desired_yaw = atan2(dir(1), dir(0));
    double yaw_err = desired_yaw - yaw;
    while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;
    if (yaw_err > M_PI / 2.0) yaw_err -= M_PI;
    else if (yaw_err < -M_PI / 2.0) yaw_err += M_PI;

    wz = std::clamp(cfg_.yaw_align_kp * yaw_err,
                    -cfg_.max_align_wz, cfg_.max_align_wz);

    // 对齐后恒速直穿；未对齐先爬行转向（斜着冲窄口等于加宽自己）
    double speed = std::fabs(yaw_err) < cfg_.corridor_align_tol
                       ? cfg_.corridor_speed
                       : cfg_.corridor_align_speed;

    // 最小转弯半径: 移动中转向不能太急。原实现 v=0.4 时 wz 可达 2.5 → R=0.16m,
    // 等于在坡脚/台阶沿原地硬掰, 轮子别在 lip 上打滑蹭（实测卡死）。按
    // R_min = v/|wz| 反推角速度上限, 让转弯弧线半径不小于 min_turn_radius。
    // min_turn_radius <= 0 表示不限制 (测试/兼容)。
    if (cfg_.min_turn_radius > 1e-6)
    {
        double wz_cap = speed / cfg_.min_turn_radius;
        wz = std::clamp(wz, -wz_cap, wz_cap);
    }

    v_odom = dir * speed;
    return true;
}

} // namespace sentry_planner
