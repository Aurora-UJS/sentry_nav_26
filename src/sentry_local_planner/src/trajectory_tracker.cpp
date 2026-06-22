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
    const Eigen::Vector2d &pos, const Eigen::Vector2d &vel, double yaw,
    const MincoTrajectory &traj, double elapsed, bool has_traj)
{
    geometry_msgs::msg::Twist cmd;

    if (!has_traj)
    {
        cmd.angular.z = limitYawRate(cfg_.spin_rate);
        return cmd;
    }

    // MPC
    auto ref_func = [&traj](double t) -> Eigen::Vector4d {
        Eigen::Vector4d ref;
        ref.head(2) = traj.getPosition(t);
        ref.tail(2) = traj.getVelocity(t);
        return ref;
    };

    double ref_time = std::min(elapsed + cfg_.lookahead_time, traj.getDuration());
    Eigen::Vector2d acc_cmd = mpc_.compute(pos, vel, ref_func, elapsed);
    Eigen::Vector2d pos_ref = traj.getPosition(ref_time);
    Eigen::Vector2d vel_ref = traj.getVelocity(ref_time);
    double dt_lookahead = std::max(cfg_.lookahead_time, cfg_.ctrl_dt);
    Eigen::Vector2d vel_cmd_odom =
        vel_ref + cfg_.tracking_kp * (pos_ref - pos) + acc_cmd * dt_lookahead;

    // 限速
    if (vel_cmd_odom.norm() > cfg_.max_vel)
        vel_cmd_odom = vel_cmd_odom.normalized() * cfg_.max_vel;

    // Odom → body frame
    double cy = cos(-yaw), sy = sin(-yaw);
    cmd.linear.x = cy * vel_cmd_odom(0) - sy * vel_cmd_odom(1);
    cmd.linear.y = sy * vel_cmd_odom(0) + cy * vel_cmd_odom(1);
    cmd.angular.z = limitYawRate(computeAngularVelocity(pos, yaw, traj, elapsed));

    return cmd;
}

double TrajectoryTracker::computeAngularVelocity(
    const Eigen::Vector2d &pos, double yaw,
    const MincoTrajectory &traj, double elapsed)
{
    bool align_to_path = cfg_.follow_path_yaw;
    if (!align_to_path && env_ && env_->isInMap(pos))
    {
        double center_dist = env_->getDistance(pos);
        align_to_path = center_dist < cfg_.narrow_passage_dist;
    }

    if (align_to_path)
    {
        double ref_time = std::min(elapsed + cfg_.lookahead_time, traj.getDuration());
        Eigen::Vector2d vel_dir = traj.getVelocity(ref_time);
        if (vel_dir.norm() > 0.1)
        {
            double desired_yaw = atan2(vel_dir(1), vel_dir(0));
            double yaw_err = desired_yaw - yaw;
            while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
            while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

            return cfg_.yaw_align_kp * yaw_err;
        }
        return 0.0;
    }

    return cfg_.spin_rate;
}

double TrajectoryTracker::limitYawRate(double target_yaw_rate)
{
    target_yaw_rate = std::clamp(target_yaw_rate, -cfg_.max_yaw_rate, cfg_.max_yaw_rate);

    if (!has_yaw_rate_cmd_)
    {
        last_yaw_rate_cmd_ = target_yaw_rate;
        has_yaw_rate_cmd_ = true;
        return last_yaw_rate_cmd_;
    }

    double max_delta = std::max(0.0, cfg_.max_yaw_acc * cfg_.ctrl_dt);
    double delta = std::clamp(target_yaw_rate - last_yaw_rate_cmd_, -max_delta, max_delta);
    last_yaw_rate_cmd_ += delta;

    if (std::abs(target_yaw_rate) < 1e-3 && std::abs(last_yaw_rate_cmd_) < max_delta)
        last_yaw_rate_cmd_ = 0.0;

    return last_yaw_rate_cmd_;
}

} // namespace sentry_planner
