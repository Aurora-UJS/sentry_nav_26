#pragma once
/**
 * TrajectoryTracker: MPC 轨迹跟踪 + 自转/窄口航向对齐
 *
 * 纯算法类，不依赖 ROS。输入当前状态和轨迹，输出 cmd_vel。
 */

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <plan_env/environment_interface.hpp>
#include <sentry_local_planner/minco_trajectory.hpp>
#include <sentry_local_planner/mpc_controller.hpp>

namespace sentry_planner
{

class TrajectoryTracker
{
public:
    struct Config
    {
        double max_vel = 3.0;
        double max_acc = 3.0;
        double ctrl_dt = 0.02;
        double spin_rate = 3.0;
        double narrow_passage_dist = 0.5;
        double yaw_align_kp = 3.0;
        int mpc_horizon = 10;
        double mpc_q_pos = 10.0;
        double mpc_q_vel = 1.0;
        double mpc_r_acc = 0.1;
        // true: cmd.linear 保持 odom/世界系，由下游（chassis_cmd_node / 电控 MCU）
        // 用高频陀螺 yaw 旋转到机体系 —— 避免用带龄期的 LIO yaw 在 50Hz 侧旋转
        bool world_frame_cmd = false;
    };

    void init(const Config &cfg, sentry_nav::EnvironmentInterface *env);

    /**
     * 计算 cmd_vel
     * @param pos       当前位置 (odom 系)
     * @param vel       当前速度 (odom 系)
     * @param yaw       当前航向
     * @param traj      MINCO 轨迹
     * @param elapsed   轨迹已过时间
     * @param has_traj  是否有有效轨迹
     * @return          cmd_vel (body frame)
     */
    geometry_msgs::msg::Twist compute(
        const Eigen::Vector2d &pos, const Eigen::Vector2d &vel, double yaw,
        const MincoTrajectory &traj, double elapsed, bool has_traj);

private:
    double computeAngularVelocity(
        const Eigen::Vector2d &pos, double yaw,
        const MincoTrajectory &traj, double elapsed);

    Config cfg_;
    MPCController mpc_;
    sentry_nav::EnvironmentInterface *env_ = nullptr;
};

} // namespace sentry_planner
