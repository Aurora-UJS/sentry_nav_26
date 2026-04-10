#pragma once
/**
 * MincoTrajectory: 分段五次多项式轨迹 + L-BFGS 梯度优化
 *
 * 输入: A* waypoints + 起终点速度/加速度
 * 输出: 任意时刻的 position, velocity, acceleration
 *
 * 每段: p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
 * 优化变量: 中间路径点坐标 (起终点固定)
 * 代价函数: J_smooth + J_collision(ESDF+动态障碍) + J_feasibility
 * 求解器: NLopt L-BFGS
 */

#include <Eigen/Eigen>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <nlopt.hpp>
#include <plan_env/edt_environment.hpp>

namespace sentry_planner
{

class MincoTrajectory
{
public:
    struct Segment
    {
        double duration;
        // coeffs: 6 rows (c0..c5) x 2 cols (x, y)
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

    /**
     * 配置梯度优化器 (在节点初始化时调用一次)
     */
    void setOptimizer(fast_planner::EDTEnvironment* env,
                      double lambda_smooth, double lambda_col, double lambda_feas,
                      double dist0, double max_vel, double max_acc,
                      double robot_radius = 0.3,
                      int num_samples = 8, int max_iter = 200, double max_time_s = 0.02)
    {
        edt_env_       = env;
        lambda_smooth_ = lambda_smooth;
        lambda_col_    = lambda_col;
        lambda_feas_   = lambda_feas;
        dist0_         = dist0;
        max_vel_       = max_vel;
        max_acc_       = max_acc;
        robot_radius_  = robot_radius;
        num_samples_   = num_samples;
        max_iter_      = max_iter;
        max_time_s_    = max_time_s;

        // 预计算 footprint 偏移: 中心 + 前后左右
        footprint_offsets_ = {
            Eigen::Vector2d(0, 0),
            Eigen::Vector2d( robot_radius_, 0),
            Eigen::Vector2d(-robot_radius_, 0),
            Eigen::Vector2d(0,  robot_radius_),
            Eigen::Vector2d(0, -robot_radius_),
        };
    }

    /**
     * 从 waypoints 构建轨迹，若已配置优化器则执行 L-BFGS 优化
     * @param traj_start_time  轨迹开始的绝对时间 (秒)，用于动态障碍预测；<0 表示只做静态 ESDF
     */
    void setup(const std::vector<Eigen::Vector2d> &waypoints,
               const Eigen::Vector2d &start_vel, const Eigen::Vector2d &start_acc,
               const Eigen::Vector2d &end_vel, const Eigen::Vector2d &end_acc,
               double max_vel, double max_acc,
               double traj_start_time = -1.0)
    {
        int N = (int)waypoints.size() - 1;
        if (N < 1) return;

        // 保存边界条件供 rebuild 时使用
        start_vel_ = start_vel;
        start_acc_ = start_acc;
        end_vel_   = end_vel;
        end_acc_   = end_acc;
        traj_start_time_ = traj_start_time;

        segments_.resize(N);
        total_duration_ = 0;

        // --- 时间分配: 全程梯形速度剖面，按段距离比例分配 ---
        durations_opt_.resize(N);
        double total_dist = 0;
        std::vector<double> seg_dist(N);
        for (int i = 0; i < N; ++i)
        {
            seg_dist[i] = (waypoints[i + 1] - waypoints[i]).norm();
            total_dist += seg_dist[i];
        }

        double total_time = total_dist / max_vel + max_vel / max_acc;
        total_time = std::max(total_time, 0.5);

        for (int i = 0; i < N; ++i)
        {
            durations_opt_[i] = std::max(total_time * seg_dist[i] / total_dist, 0.05);
            total_duration_ += durations_opt_[i];
        }

        // 闭式初始解
        rebuildFromWaypoints(waypoints);

        // --- L-BFGS 优化阶段 ---
        if (edt_env_ != nullptr && (int)waypoints.size() >= 3)
        {
            runOptimization(waypoints);
        }
    }

    double getDuration() const { return total_duration_; }
    bool empty() const { return segments_.empty(); }
    int numSegments() const { return (int)segments_.size(); }

    Eigen::Vector2d getPosition(double t) const
    {
        int seg;
        double local_t;
        locateSegment(t, seg, local_t);
        return segments_[seg].pos(local_t);
    }

    Eigen::Vector2d getVelocity(double t) const
    {
        int seg;
        double local_t;
        locateSegment(t, seg, local_t);
        return segments_[seg].vel(local_t);
    }

    Eigen::Vector2d getAcceleration(double t) const
    {
        int seg;
        double local_t;
        locateSegment(t, seg, local_t);
        return segments_[seg].acc(local_t);
    }

private:
    std::vector<Segment> segments_;
    double total_duration_ = 0;

    // 边界条件 (setup 时保存)
    Eigen::Vector2d start_vel_, start_acc_, end_vel_, end_acc_;
    std::vector<double> durations_opt_;
    double traj_start_time_ = -1.0;

    // 优化器配置
    fast_planner::EDTEnvironment* edt_env_ = nullptr;
    double lambda_smooth_ = 1.0;
    double lambda_col_    = 8.0;
    double lambda_feas_   = 0.001;
    double dist0_         = 0.5;
    double max_vel_       = 3.0;
    double max_acc_       = 3.0;
    double robot_radius_  = 0.3;
    int    num_samples_   = 8;
    int    max_iter_      = 200;
    double max_time_s_    = 0.02;
    std::vector<Eigen::Vector2d> footprint_offsets_;

    // 优化内部状态
    Eigen::Vector2d q0_fixed_, qN_fixed_;
    std::vector<double> best_var_;
    double min_cost_ = 1e18;
    bool coarse_stage_ = false;  // 两阶段优化: true=粗优化(垂直梯度), false=细优化(全梯度)

    // ==================== 优化核心 ====================

    void runOptimization(const std::vector<Eigen::Vector2d> &waypoints)
    {
        int N = (int)segments_.size();
        int M = N - 1; // 中间路径点数
        if (M < 1) return;

        q0_fixed_ = waypoints.front();
        qN_fixed_ = waypoints.back();

        // 初始化优化变量: x = [q1.x, q1.y, ..., q_{M}.x, q_{M}.y]
        std::vector<double> x(2 * M);
        for (int k = 0; k < M; ++k)
        {
            x[2*k]   = waypoints[k+1](0);
            x[2*k+1] = waypoints[k+1](1);
        }
        best_var_ = x;
        min_cost_ = 1e18;

        // 盒约束: 每个路径点在初始值 ±5m 范围内
        std::vector<double> lb(2*M), ub(2*M);
        for (int i = 0; i < 2*M; ++i)
        {
            lb[i] = x[i] - 5.0;
            ub[i] = x[i] + 5.0;
        }

        try
        {
            // Stage 1: 粗优化 — 碰撞梯度只保留垂直于轨迹方向的分量
            // 确定轨迹整体形状，避免沿轨迹方向推移导致异常速度
            {
                coarse_stage_ = true;
                nlopt::opt opt(nlopt::LD_LBFGS, 2 * M);
                opt.set_min_objective(nloptCallback, this);
                opt.set_maxeval(max_iter_ / 2);
                opt.set_maxtime(max_time_s_ * 0.4);
                opt.set_xtol_rel(1e-3);
                opt.set_lower_bounds(lb);
                opt.set_upper_bounds(ub);
                double cost;
                opt.optimize(x, cost);
            }
        }
        catch (const std::exception &) {}

        // 用 Stage 1 最优解作为 Stage 2 起点
        x = best_var_;
        min_cost_ = 1e18;

        try
        {
            // Stage 2: 细优化 — 使用完整碰撞梯度，精调轨迹
            {
                coarse_stage_ = false;
                nlopt::opt opt(nlopt::LD_LBFGS, 2 * M);
                opt.set_min_objective(nloptCallback, this);
                opt.set_maxeval(max_iter_);
                opt.set_maxtime(max_time_s_ * 0.6);
                opt.set_xtol_rel(1e-4);
                opt.set_lower_bounds(lb);
                opt.set_upper_bounds(ub);
                double cost;
                opt.optimize(x, cost);
            }
        }
        catch (const std::exception &) {}

        // 用最优解重建轨迹
        std::vector<Eigen::Vector2d> opt_wp;
        opt_wp.push_back(q0_fixed_);
        for (int k = 0; k < M; ++k)
            opt_wp.push_back(Eigen::Vector2d(best_var_[2*k], best_var_[2*k+1]));
        opt_wp.push_back(qN_fixed_);
        rebuildFromWaypoints(opt_wp);
    }

    static double nloptCallback(const std::vector<double> &x,
                                 std::vector<double> &grad,
                                 void *data)
    {
        MincoTrajectory *self = static_cast<MincoTrajectory*>(data);
        return self->computeCostAndGrad(x, grad);
    }

    double computeCostAndGrad(const std::vector<double> &x, std::vector<double> &grad)
    {
        int N = (int)segments_.size();
        int M = N - 1;

        // 解码当前路径点并重建轨迹
        std::vector<Eigen::Vector2d> cur_wp;
        cur_wp.push_back(q0_fixed_);
        for (int k = 0; k < M; ++k)
            cur_wp.push_back(Eigen::Vector2d(x[2*k], x[2*k+1]));
        cur_wp.push_back(qN_fixed_);
        rebuildFromWaypoints(cur_wp);

        grad.assign(2 * M, 0.0);

        double cost = 0.0;
        cost += lambda_smooth_ * calcSmoothCost(grad);
        cost += lambda_col_    * calcCollisionCost(grad);
        cost += lambda_feas_   * calcFeasibilityCost(grad);

        // 记录最优解
        if (cost < min_cost_)
        {
            min_cost_ = cost;
            best_var_ = x;
        }

        return cost;
    }

    /**
     * 用给定路径点重建所有段的多项式系数
     * 中间点速度/加速度通过有限差分估计
     */
    void rebuildFromWaypoints(const std::vector<Eigen::Vector2d> &wp)
    {
        int N = (int)segments_.size();

        std::vector<Eigen::Vector2d> vels(N + 1), accs(N + 1);
        vels[0] = start_vel_;
        vels[N] = end_vel_;
        accs[0] = start_acc_;
        accs[N] = end_acc_;

        for (int i = 1; i < N; ++i)
        {
            Eigen::Vector2d d_prev = (wp[i] - wp[i-1]).normalized();
            Eigen::Vector2d d_next = (wp[i+1] - wp[i]).normalized();
            double v_mag = std::min(max_vel_ * 0.8,
                                    (wp[i+1] - wp[i]).norm() / durations_opt_[i]);
            Eigen::Vector2d dir = d_prev + d_next;
            if (dir.norm() > 1e-6) dir.normalize();
            else dir = d_next;
            vels[i] = dir * v_mag;
        }

        for (int i = 1; i < N; ++i)
        {
            double dt = 0.5 * (durations_opt_[i-1] + durations_opt_[i]);
            accs[i] = (vels[i+1 < N+1 ? i+1 : i] - vels[i > 0 ? i-1 : i]) / dt;
            if (accs[i].norm() > max_acc_)
                accs[i] = accs[i].normalized() * max_acc_;
        }

        for (int i = 0; i < N; ++i)
        {
            segments_[i].duration = durations_opt_[i];
            solveQuintic(wp[i], vels[i], accs[i],
                         wp[i+1], vels[i+1], accs[i+1],
                         durations_opt_[i], segments_[i].coeffs);
        }
    }

    /**
     * 计算 A^{-1} 的第一行 [α,β,γ]，解方程 A*[α,β,γ]^T = [1,0,0]^T
     * 用于将碰撞梯度从采样点反传到路径点
     *
     * A = [[T^3, T^4, T^5], [3T^2, 4T^3, 5T^4], [6T, 12T^2, 20T^3]]
     * 当 p1 变化 Δ 时: b(0) 变化 Δ → [c3,c4,c5] 变化 A^{-1} * [Δ,0,0]^T = [α,β,γ]*Δ
     */
    static Eigen::Vector3d solveBackpropCoeffs(double T)
    {
        double T2 = T*T, T3 = T2*T, T4 = T3*T, T5 = T4*T;
        Eigen::Matrix3d A;
        A << T3,   T4,    T5,
             3*T2, 4*T3,  5*T4,
             6*T,  12*T2, 20*T3;
        Eigen::Vector3d e1(1.0, 0.0, 0.0);
        return A.colPivHouseholderQr().solve(e1);
    }

    // ==================== 代价函数 ====================

    /**
     * 平滑代价: 加速度积分 ∫||a(t)||^2 dt
     * 梯度通过 A^{-1} 反传到中间路径点
     */
    double calcSmoothCost(std::vector<double> &grad_x)
    {
        double cost = 0.0;
        int N = (int)segments_.size();
        int M = N - 1;

        for (int i = 0; i < N; ++i)
        {
            double Ti = durations_opt_[i];
            Eigen::Vector3d bc = solveBackpropCoeffs(Ti);
            double alpha = bc(0), beta = bc(1), gamma = bc(2);
            double dt = Ti / num_samples_;

            for (int k = 0; k < num_samples_; ++k)
            {
                double tau = (k + 0.5) * dt;
                Eigen::Vector2d a = segments_[i].acc(tau);
                cost += a.squaredNorm() * dt;

                // ∂(||a||^2·dt)/∂c3 = 2*a*6τ*dt, etc.
                // ∂c3/∂q_i = α, ∂c4/∂q_i = β, ∂c5/∂q_i = γ
                // ∂acc/∂q_i = (6τ*α + 12τ^2*β + 20τ^3*γ)
                double dpq_end = (6*tau*alpha + 12*tau*tau*beta + 20*tau*tau*tau*gamma) * dt;
                // ∂acc/∂q_{i-1}: c0 不影响 acc；c3,c4,c5 对 q_{i-1} 的偏导符号相反
                double dpq_start = -dpq_end;

                Eigen::Vector2d dJda = 2.0 * a;

                int end_idx   = i;     // 第 i 段终点是路径点 i (0-indexed，含起点)
                int start_idx = i - 1; // 第 i 段起点是路径点 i-1

                // 映射到 grad_x 索引 (中间路径点 1..M，grad_x 索引 0..2M-1)
                if (end_idx > 0 && end_idx < N)
                {
                    grad_x[2*(end_idx-1)]   += dpq_end * dJda(0);
                    grad_x[2*(end_idx-1)+1] += dpq_end * dJda(1);
                }
                if (start_idx > 0 && start_idx < M)
                {
                    grad_x[2*(start_idx-1)]   += dpq_start * dJda(0);
                    grad_x[2*(start_idx-1)+1] += dpq_start * dJda(1);
                }
            }
        }
        return cost;
    }

    /**
     * 碰撞代价: ESDF 静态 + 动态障碍预测位置
     * 静态: evaluateEDTWithGrad (解析梯度)
     * 动态: evaluateCoarseEDT (有限差分梯度)
     */
    double calcCollisionCost(std::vector<double> &grad_x)
    {
        double cost = 0.0;
        int N = (int)segments_.size();
        int M = N - 1;

        double t_elapsed = 0.0;

        for (int i = 0; i < N; ++i)
        {
            double Ti = durations_opt_[i];
            Eigen::Vector3d bc = solveBackpropCoeffs(Ti);
            double alpha = bc(0), beta = bc(1), gamma = bc(2);
            double dt = Ti / num_samples_;

            for (int k = 0; k < num_samples_; ++k)
            {
                double tau = (k + 0.5) * dt;
                Eigen::Vector2d pos = segments_[i].pos(tau);

                // 反传系数 (只依赖段时长，与 footprint 无关)
                // dp(τ)/dq_end = τ³α + τ⁴β + τ⁵γ
                // dp(τ)/dq_start = 1 - dp(τ)/dq_end (因 c0=q_start 贡献 "1")
                double dpq_end   = tau*tau*tau * alpha + tau*tau*tau*tau * beta +
                                   tau*tau*tau*tau*tau * gamma;
                double dpq_start = 1.0 - dpq_end;

                // 查 footprint 上所有点，取距离最小的那个产生碰撞代价
                for (const auto &offset : footprint_offsets_)
                {
                    Eigen::Vector2d fp = pos + offset;
                    double dist;
                    Eigen::Vector2d esdf_grad;

                    if (traj_start_time_ >= 0.0 && edt_env_->hasDynamicObjects())
                    {
                        double t_abs = traj_start_time_ + t_elapsed + tau;
                        dist = edt_env_->evaluateCoarseEDT(fp, t_abs);
                        const double fd_eps = 0.01;
                        Eigen::Vector2d px(fp(0)+fd_eps, fp(1)), mx(fp(0)-fd_eps, fp(1));
                        Eigen::Vector2d py(fp(0), fp(1)+fd_eps), my(fp(0), fp(1)-fd_eps);
                        esdf_grad(0) = (edt_env_->evaluateCoarseEDT(px, t_abs) -
                                        edt_env_->evaluateCoarseEDT(mx, t_abs)) / (2.0*fd_eps);
                        esdf_grad(1) = (edt_env_->evaluateCoarseEDT(py, t_abs) -
                                        edt_env_->evaluateCoarseEDT(my, t_abs)) / (2.0*fd_eps);
                    }
                    else
                    {
                        edt_env_->evaluateEDTWithGrad(fp, -1.0, dist, esdf_grad);
                    }

                    if (dist < dist0_)
                    {
                        double pen = dist - dist0_;
                        cost += pen * pen;

                        // 粗优化: 碰撞梯度去除沿轨迹方向分量，只保留垂直分量
                        Eigen::Vector2d grad_used = esdf_grad;
                        if (coarse_stage_)
                        {
                            Eigen::Vector2d vel = segments_[i].vel(tau);
                            double vn = vel.norm();
                            if (vn > 1e-6)
                            {
                                Eigen::Vector2d tangent = vel / vn;
                                grad_used -= grad_used.dot(tangent) * tangent;
                            }
                        }

                        Eigen::Vector2d dJdpos = 2.0 * pen * grad_used;

                        int end_idx   = i;
                        int start_idx = i - 1;
                        if (end_idx > 0 && end_idx < N)
                        {
                            grad_x[2*(end_idx-1)]   += dpq_end * dJdpos(0);
                            grad_x[2*(end_idx-1)+1] += dpq_end * dJdpos(1);
                        }
                        if (start_idx > 0 && start_idx < M)
                        {
                            grad_x[2*(start_idx-1)]   += dpq_start * dJdpos(0);
                            grad_x[2*(start_idx-1)+1] += dpq_start * dJdpos(1);
                        }
                    }
                }
            }
            t_elapsed += Ti;
        }
        return cost;
    }

    /**
     * 可行性代价: 速度/加速度超限的罚函数
     */
    double calcFeasibilityCost(std::vector<double> &grad_x)
    {
        double cost = 0.0;
        int N = (int)segments_.size();
        int M = N - 1;
        double vm2 = max_vel_ * max_vel_;
        double am2 = max_acc_ * max_acc_;

        for (int i = 0; i < N; ++i)
        {
            double Ti = durations_opt_[i];
            Eigen::Vector3d bc = solveBackpropCoeffs(Ti);
            double alpha = bc(0), beta = bc(1), gamma = bc(2);
            double dt = Ti / num_samples_;

            for (int k = 0; k < num_samples_; ++k)
            {
                double tau = (k + 0.5) * dt;
                Eigen::Vector2d v = segments_[i].vel(tau);
                Eigen::Vector2d a = segments_[i].acc(tau);

                // 速度超限: ||v||^2 - vm^2
                double vd = v.squaredNorm() - vm2;
                if (vd > 0)
                {
                    cost += vd * vd;
                    Eigen::Vector2d dJdv = 4.0 * vd * v;

                    // ∂vel/∂q_i: vel 含 c3,c4,c5 项
                    // vel(τ) = c1 + 2c2τ + 3c3τ^2 + 4c4τ^3 + 5c5τ^4
                    // ∂c3/∂q_i = α 等
                    double dpq_end   = (3*tau*tau*alpha + 4*tau*tau*tau*beta +
                                        5*tau*tau*tau*tau*gamma) * dt;
                    double dpq_start = -dpq_end;

                    int end_idx   = i;
                    int start_idx = i - 1;
                    if (end_idx > 0 && end_idx < N)
                    {
                        grad_x[2*(end_idx-1)]   += dpq_end * dJdv(0);
                        grad_x[2*(end_idx-1)+1] += dpq_end * dJdv(1);
                    }
                    if (start_idx > 0 && start_idx < M)
                    {
                        grad_x[2*(start_idx-1)]   += dpq_start * dJdv(0);
                        grad_x[2*(start_idx-1)+1] += dpq_start * dJdv(1);
                    }
                }

                // 加速度超限: ||a||^2 - am^2
                double ad = a.squaredNorm() - am2;
                if (ad > 0)
                {
                    cost += ad * ad;
                    Eigen::Vector2d dJda = 4.0 * ad * a;

                    double dpq_end   = (6*tau*alpha + 12*tau*tau*beta +
                                        20*tau*tau*tau*gamma) * dt;
                    double dpq_start = -dpq_end;

                    int end_idx   = i;
                    int start_idx = i - 1;
                    if (end_idx > 0 && end_idx < N)
                    {
                        grad_x[2*(end_idx-1)]   += dpq_end * dJda(0);
                        grad_x[2*(end_idx-1)+1] += dpq_end * dJda(1);
                    }
                    if (start_idx > 0 && start_idx < M)
                    {
                        grad_x[2*(start_idx-1)]   += dpq_start * dJda(0);
                        grad_x[2*(start_idx-1)+1] += dpq_start * dJda(1);
                    }
                }
            }
        }
        return cost;
    }

    // ==================== 工具函数 ====================

    void locateSegment(double t, int &seg, double &local_t) const
    {
        t = std::max(0.0, std::min(t, total_duration_));
        double acc = 0;
        for (int i = 0; i < (int)segments_.size(); ++i)
        {
            if (acc + segments_[i].duration >= t || i == (int)segments_.size() - 1)
            {
                seg = i;
                local_t = t - acc;
                return;
            }
            acc += segments_[i].duration;
        }
        seg = (int)segments_.size() - 1;
        local_t = segments_.back().duration;
    }

    static void solveQuintic(
        const Eigen::Vector2d &p0, const Eigen::Vector2d &v0, const Eigen::Vector2d &a0,
        const Eigen::Vector2d &p1, const Eigen::Vector2d &v1, const Eigen::Vector2d &a1,
        double T, Eigen::Matrix<double, 6, 2> &coeffs)
    {
        double T2 = T*T, T3 = T2*T, T4 = T3*T, T5 = T4*T;

        for (int d = 0; d < 2; ++d)
        {
            double c0 = p0(d);
            double c1 = v0(d);
            double c2 = a0(d) / 2.0;

            Eigen::Matrix3d A;
            A << T3, T4, T5,
                 3*T2, 4*T3, 5*T4,
                 6*T, 12*T2, 20*T3;

            Eigen::Vector3d b;
            b(0) = p1(d) - c0 - c1*T - c2*T2;
            b(1) = v1(d) - c1 - 2*c2*T;
            b(2) = a1(d) - 2*c2;

            Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);

            coeffs(0, d) = c0;
            coeffs(1, d) = c1;
            coeffs(2, d) = c2;
            coeffs(3, d) = x(0);
            coeffs(4, d) = x(1);
            coeffs(5, d) = x(2);
        }
    }
};

} // namespace sentry_planner
