#pragma once
/**
 * MincoTrajectory: 分段五次多项式最小 jerk 轨迹
 *
 * 输入: A* waypoints + 起终点速度/加速度
 * 输出: 任意时刻的 position, velocity, acceleration
 *
 * 每段: p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
 * 边界条件: pos/vel/acc 连续
 * 自由度用于最小化 jerk (三阶导数的积分)
 */

#include <Eigen/Eigen>
#include <vector>
#include <cmath>
#include <algorithm>

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
     * 从 waypoints 构建轨迹
     * @param waypoints  A* 采样点 (含起终点)
     * @param start_vel  起点速度
     * @param start_acc  起点加速度
     * @param end_vel    终点速度
     * @param end_acc    终点加速度
     * @param max_vel    最大速度 (用于时间分配)
     * @param max_acc    最大加速度
     */
    void setup(const std::vector<Eigen::Vector2d> &waypoints,
               const Eigen::Vector2d &start_vel, const Eigen::Vector2d &start_acc,
               const Eigen::Vector2d &end_vel, const Eigen::Vector2d &end_acc,
               double max_vel, double max_acc)
    {
        int N = (int)waypoints.size() - 1; // number of segments
        if (N < 1) return;

        segments_.resize(N);
        total_duration_ = 0;

        // --- 时间分配: 全程梯形速度剖面，按段距离比例分配 ---
        std::vector<double> durations(N);
        double total_dist = 0;
        std::vector<double> seg_dist(N);
        for (int i = 0; i < N; ++i)
        {
            seg_dist[i] = (waypoints[i + 1] - waypoints[i]).norm();
            total_dist += seg_dist[i];
        }

        // 全程梯形剖面时间: t = dist/v_max + v_max/a_max
        double total_time = total_dist / max_vel + max_vel / max_acc;
        total_time = std::max(total_time, 0.5); // 最小 0.5s

        for (int i = 0; i < N; ++i)
        {
            // 按距离比例分配时间，短段也有最小时间
            durations[i] = std::max(total_time * seg_dist[i] / total_dist, 0.05);
            total_duration_ += durations[i];
        }

        // --- 计算中间点的速度和加速度 (有限差分) ---
        std::vector<Eigen::Vector2d> vels(N + 1), accs(N + 1);
        vels[0] = start_vel;
        vels[N] = end_vel;
        accs[0] = start_acc;
        accs[N] = end_acc;

        // 中间点速度: 两侧段方向的加权平均
        for (int i = 1; i < N; ++i)
        {
            Eigen::Vector2d d_prev = (waypoints[i] - waypoints[i - 1]).normalized();
            Eigen::Vector2d d_next = (waypoints[i + 1] - waypoints[i]).normalized();
            double v_mag = std::min(max_vel * 0.8,
                                     (waypoints[i + 1] - waypoints[i]).norm() / durations[i]);
            // 方向: 前后段的平均方向
            Eigen::Vector2d dir = (d_prev + d_next);
            if (dir.norm() > 1e-6)
                dir.normalize();
            else
                dir = d_next;
            vels[i] = dir * v_mag;
        }

        // 中间点加速度: 速度的有限差分
        for (int i = 1; i < N; ++i)
        {
            double dt = 0.5 * (durations[i - 1] + durations[i]);
            accs[i] = (vels[i + 1 < N + 1 ? i + 1 : i] - vels[i > 0 ? i - 1 : i]) / dt;
            // 限幅
            if (accs[i].norm() > max_acc)
                accs[i] = accs[i].normalized() * max_acc;
        }

        // --- 每段求解五次多项式系数 ---
        for (int i = 0; i < N; ++i)
        {
            segments_[i].duration = durations[i];
            solveQuintic(waypoints[i], vels[i], accs[i],
                         waypoints[i + 1], vels[i + 1], accs[i + 1],
                         durations[i], segments_[i].coeffs);
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

    /**
     * 五次多项式求解:
     * 给定 p(0)=p0, p'(0)=v0, p''(0)=a0, p(T)=p1, p'(T)=v1, p''(T)=a1
     * 求 c0..c5
     */
    static void solveQuintic(
        const Eigen::Vector2d &p0, const Eigen::Vector2d &v0, const Eigen::Vector2d &a0,
        const Eigen::Vector2d &p1, const Eigen::Vector2d &v1, const Eigen::Vector2d &a1,
        double T, Eigen::Matrix<double, 6, 2> &coeffs)
    {
        // c0 = p0, c1 = v0, c2 = a0/2
        // c3, c4, c5 从终点条件解出
        double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;

        for (int d = 0; d < 2; ++d)
        {
            double c0 = p0(d);
            double c1 = v0(d);
            double c2 = a0(d) / 2.0;

            // 终点条件形成 3x3 线性系统
            // p(T) = c0 + c1*T + c2*T^2 + c3*T^3 + c4*T^4 + c5*T^5 = p1
            // p'(T) = c1 + 2*c2*T + 3*c3*T^2 + 4*c4*T^3 + 5*c5*T^4 = v1
            // p''(T) = 2*c2 + 6*c3*T + 12*c4*T^2 + 20*c5*T^3 = a1

            Eigen::Matrix3d A;
            A << T3, T4, T5,
                 3*T2, 4*T3, 5*T4,
                 6*T, 12*T2, 20*T3;

            Eigen::Vector3d b;
            b(0) = p1(d) - c0 - c1 * T - c2 * T2;
            b(1) = v1(d) - c1 - 2 * c2 * T;
            b(2) = a1(d) - 2 * c2;

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
