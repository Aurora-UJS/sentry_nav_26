#pragma once
/**
 * MPCController: 线性 MPC 轨迹跟踪控制器
 *
 * 模型: 2D 双积分器 (麦轮全向底盘)
 *   状态 x = [px, py, vx, vy]  (4维)
 *   输入 u = [ax, ay]          (2维)
 *   x(k+1) = A*x(k) + B*u(k)
 *
 * 批量 QP 求解:
 *   X = S*x0 + T*U
 *   J = (X-Xref)'*Q_bar*(X-Xref) + U'*R_bar*U
 *   U* = (T'*Q_bar*T + R_bar)^{-1} * T'*Q_bar*(Xref - S*x0)
 *   只取第一个控制输入
 *
 * 性能: S、T、Q_bar、T'Q 以及 H=(T'QT+R) 的 LDLT 分解只依赖配置 (N, dt, 权重)
 * 与常量模型 (Ad_, Bd_), 与运行时状态无关, 故在 buildModel() 中一次性构建并缓存;
 * compute() 每拍只构造 Xref/x0 并解一次三角方程 (build once / solve per tick)。
 */

#include <Eigen/Eigen>
#include <functional>
#include <cmath>
#include <algorithm>

namespace sentry_planner
{

class MPCController
{
public:
    struct Config
    {
        int horizon = 10;        // 预测步数
        double dt = 0.05;        // 离散时间步长 (与控制频率一致)
        double q_pos = 10.0;     // 位置误差权重
        double q_vel = 1.0;      // 速度误差权重
        double r_acc = 0.1;      // 控制量 (加速度) 权重
        double max_vel = 3.0;    // 速度限幅 m/s
        double max_acc = 3.0;    // 加速度限幅 m/s^2
    };

    MPCController() { buildModel(); }

    void setConfig(const Config &cfg)
    {
        config_ = cfg;
        buildModel();
    }

    /**
     * 计算加速度控制量 (双积分器模型)
     *
     * 状态 x = [px, py, vx, vy]  输入 u = [ax, ay]
     * x(k+1) = A*x(k) + B*u(k)
     *
     * @return 加速度 [ax, ay] (odom 系) + 预测下一步速度
     */
    Eigen::Vector2d compute(
        const Eigen::Vector2d &pos, const Eigen::Vector2d &vel,
        const std::function<Eigen::Vector4d(double)> &ref_func,
        double current_time)
    {
        const int N = config_.horizon;
        const int nx = 4, nu = 2;

        // 当前状态
        Eigen::Vector4d x0;
        x0 << pos(0), pos(1), vel(0), vel(1);

        // 参考轨迹 (随时间变化, 每拍重建)
        Eigen::VectorXd Xref(nx * N);
        for (int k = 0; k < N; ++k)
        {
            double t = current_time + (k + 1) * config_.dt;
            Xref.segment(k * nx, nx) = ref_func(t);
        }

        // 复用 buildModel() 中预构建的常量矩阵 (S_, TtQ_) 与 H 的 LDLT 分解:
        //   U* = (T'QT + R)^{-1} T'Q(Xref - S*x0) = H^{-1} * g
        Eigen::VectorXd g = TtQ_ * (Xref - S_ * x0);
        Eigen::VectorXd U = H_ldlt_.solve(g);

        Eigen::Vector2d u_opt = U.head(nu);

        // 限幅
        if (u_opt.norm() > config_.max_acc)
            u_opt = u_opt.normalized() * config_.max_acc;

        return u_opt;
    }

private:
    Config config_;
    Eigen::Matrix4d Ad_; // 离散状态转移矩阵
    Eigen::Matrix<double, 4, 2> Bd_; // 离散输入矩阵

    // 仅依赖配置 (N, dt, 权重) 与常量模型 (Ad_, Bd_) 的缓存量,
    // 在 buildModel() 中一次性构建, compute() 每拍直接复用。
    Eigen::MatrixXd S_;     // X_free = S_ * x0
    Eigen::MatrixXd T_;     // X_forced = T_ * U (下三角 Toeplitz)
    Eigen::MatrixXd Q_bar_; // 块对角权重 (末端块 x3)
    Eigen::MatrixXd TtQ_;   // T_' * Q_bar_, 用于构造 g
    Eigen::LDLT<Eigen::MatrixXd> H_ldlt_; // H = T_'*Q_bar_*T_ + R_bar 的 LDLT 分解

    // 构建常量模型与所有仅依赖配置的矩阵 (ctor 与 setConfig 各调用一次)。
    // 将原本 compute() 每拍重复的矩阵构建及 H 的 LDLT 分解全部前移到此,
    // 运行期 (50Hz) 只剩 Xref/x0 构建与一次三角回代求解。
    void buildModel()
    {
        double dt = config_.dt;
        double dt2 = dt * dt;
        const int N = config_.horizon;
        const int nx = 4, nu = 2;

        // 双积分器离散化:
        // px(k+1) = px(k) + vx(k)*dt + 0.5*ax*dt^2
        // vx(k+1) = vx(k) + ax*dt
        Ad_ << 1, 0, dt, 0,
               0, 1, 0,  dt,
               0, 0, 1,  0,
               0, 0, 0,  1;

        Bd_ << 0.5 * dt2, 0,
               0,         0.5 * dt2,
               dt,        0,
               0,         dt;

        // S 矩阵: X_free = S * x0
        S_.resize(nx * N, nx);
        Eigen::Matrix4d Ak = Eigen::Matrix4d::Identity();
        for (int k = 0; k < N; ++k)
        {
            Ak = Ad_ * Ak;
            S_.block(k * nx, 0, nx, nx) = Ak;
        }

        // T 矩阵: X_forced = T * U (下三角 Toeplitz)
        T_ = Eigen::MatrixXd::Zero(nx * N, nu * N);
        for (int k = 0; k < N; ++k)
        {
            Eigen::Matrix4d Aj = Eigen::Matrix4d::Identity();
            for (int j = 0; j <= k; ++j)
            {
                T_.block(k * nx, j * nu, nx, nu) = Aj * Bd_;
                Aj = Ad_ * Aj;
            }
        }

        // 权重 (块对角, 末端块 k==N-1 加权 x3)
        Eigen::VectorXd q_diag(nx);
        q_diag << config_.q_pos, config_.q_pos, config_.q_vel, config_.q_vel;
        Q_bar_ = Eigen::MatrixXd::Zero(nx * N, nx * N);
        for (int k = 0; k < N; ++k)
        {
            double weight = (k == N - 1) ? 3.0 : 1.0;
            Q_bar_.block(k * nx, k * nx, nx, nx) = weight * q_diag.asDiagonal();
        }
        Eigen::MatrixXd R_bar = config_.r_acc * Eigen::MatrixXd::Identity(nu * N, nu * N);

        // 预计算 T'Q, 以及常量 H = T'QT + R 的 LDLT 分解 (compute() 每拍复用)
        TtQ_ = T_.transpose() * Q_bar_;
        Eigen::MatrixXd H = TtQ_ * T_ + R_bar;
        H_ldlt_.compute(H);
    }
};

} // namespace sentry_planner
