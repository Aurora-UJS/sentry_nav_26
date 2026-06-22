#pragma once
/**
 * MincoTrajectory: 分段五次多项式轨迹 + L-BFGS 梯度优化
 *
 * 输入: A* waypoints + 起终点速度/加速度
 * 输出: 任意时刻的 position, velocity, acceleration
 */

#include <Eigen/Eigen>
#include <vector>
#include <cmath>
#include <algorithm>
#include <plan_env/environment_interface.hpp>

namespace sentry_planner
{

class MincoTrajectory
{
public:
    struct Segment
    {
        double duration;
        Eigen::Matrix<double, 6, 2> coeffs;

        Eigen::Vector2d pos(double t) const
        {
            Eigen::Matrix<double, 1, 6> tv;
            tv << 1, t, t*t, t*t*t, t*t*t*t, t*t*t*t*t;
            return (tv * coeffs).transpose();
        }

        Eigen::Vector2d vel(double t) const
        {
            Eigen::Matrix<double, 1, 6> tv;
            tv << 0, 1, 2*t, 3*t*t, 4*t*t*t, 5*t*t*t*t;
            return (tv * coeffs).transpose();
        }

        Eigen::Vector2d acc(double t) const
        {
            Eigen::Matrix<double, 1, 6> tv;
            tv << 0, 0, 2, 6*t, 12*t*t, 20*t*t*t;
            return (tv * coeffs).transpose();
        }
    };

    MincoTrajectory() = default;

    void setOptimizer(sentry_nav::EnvironmentInterface* env,
                      double lambda_smooth, double lambda_col, double lambda_feas,
                      double dist0, double dist0_vel_k, double max_vel, double max_acc,
                      double robot_radius = 0.3,
                      int num_samples = 8, int max_iter = 200, double max_time_s = 0.02);

    void setup(const std::vector<Eigen::Vector2d> &waypoints,
               const Eigen::Vector2d &start_vel, const Eigen::Vector2d &start_acc,
               const Eigen::Vector2d &end_vel, const Eigen::Vector2d &end_acc,
               double max_vel, double max_acc,
               double traj_start_time = -1.0);

    double getDuration() const { return total_duration_; }
    bool empty() const { return segments_.empty(); }
    int numSegments() const { return (int)segments_.size(); }

    Eigen::Vector2d getPosition(double t) const;
    Eigen::Vector2d getVelocity(double t) const;
    Eigen::Vector2d getAcceleration(double t) const;

private:
    std::vector<Segment> segments_;
    double total_duration_ = 0;

    Eigen::Vector2d start_vel_, start_acc_, end_vel_, end_acc_;
    std::vector<double> durations_opt_;
    double traj_start_time_ = -1.0;

    // 优化器配置
    sentry_nav::EnvironmentInterface* edt_env_ = nullptr;
    double lambda_smooth_ = 1.0;
    double lambda_col_    = 8.0;
    double lambda_feas_   = 0.001;
    double dist0_         = 0.5;
    double dist0_vel_k_   = 0.0;   // 速度相关裕度系数: dist0_eff = dist0 + dist0_vel_k*|v|; 0=关闭
    double max_vel_       = 3.0;
    double max_acc_       = 3.0;
    double robot_radius_  = 0.3;
    int    num_samples_   = 8;
    int    max_iter_      = 200;
    double max_time_s_    = 0.02;
    std::vector<Eigen::Vector2d> footprint_offsets_;

    Eigen::Vector2d q0_fixed_, qN_fixed_;
    std::vector<double> best_var_;
    double min_cost_ = 1e18;
    bool coarse_stage_ = false;

    void runOptimization(const std::vector<Eigen::Vector2d> &waypoints);
    static double nloptCallback(const std::vector<double> &x,
                                 std::vector<double> &grad, void *data);
    double computeCostAndGrad(const std::vector<double> &x, std::vector<double> &grad);

    void rebuildFromWaypoints(const std::vector<Eigen::Vector2d> &wp);
    static Eigen::Vector3d solveBackpropCoeffs(double T);

    double calcSmoothCost(std::vector<double> &grad_x);
    double calcCollisionCost(std::vector<double> &grad_x);
    double calcFeasibilityCost(std::vector<double> &grad_x);

    void locateSegment(double t, int &seg, double &local_t) const;
    static void solveQuintic(
        const Eigen::Vector2d &p0, const Eigen::Vector2d &v0, const Eigen::Vector2d &a0,
        const Eigen::Vector2d &p1, const Eigen::Vector2d &v1, const Eigen::Vector2d &a1,
        double T, Eigen::Matrix<double, 6, 2> &coeffs);
};

} // namespace sentry_planner
