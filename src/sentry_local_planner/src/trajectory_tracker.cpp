#include <sentry_local_planner/trajectory_tracker.hpp>
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
        cmd.angular.z = cfg_.spin_rate;
        return cmd;
    }

    // MPC
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
    cmd.angular.z = computeAngularVelocity(pos, yaw, traj, elapsed);

    return cmd;
}

double TrajectoryTracker::computeAngularVelocity(
    const Eigen::Vector2d &pos, double yaw,
    const MincoTrajectory &traj, double elapsed)
{
    if (env_ && env_->isInMap(pos))
    {
        double center_dist = env_->getDistance(pos);
        if (center_dist < cfg_.narrow_passage_dist)
        {
            Eigen::Vector2d vel_dir = traj.getVelocity(elapsed);
            if (vel_dir.norm() > 0.3)
            {
                double desired_yaw = atan2(vel_dir(1), vel_dir(0));
                double yaw_err = desired_yaw - yaw;
                while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
                while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

                if (yaw_err > M_PI / 2.0) yaw_err -= M_PI;
                else if (yaw_err < -M_PI / 2.0) yaw_err += M_PI;

                return cfg_.yaw_align_kp * yaw_err;
            }
            return 0.0;
        }
    }
    return cfg_.spin_rate;
}

} // namespace sentry_planner
