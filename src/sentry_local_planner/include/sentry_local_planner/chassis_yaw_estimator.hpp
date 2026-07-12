#pragma once
/**
 * ChassisYawEstimator: 底盘航向估计（纯逻辑，无 ROS 依赖）
 *
 * 高频信陀螺仪、低频信 LIO 的互补结构：
 *   - 陀螺仪 wz 按时间戳积分 → 毫秒级龄期的 yaw（自转 3 rad/s 下误差 ~0.2°/ms 龄期）
 *   - LIO yaw 只做零漂校准，且以 |wz| 门控 —— 实测自转时 LIO yaw 有
 *     ~50-70ms 有效龄期（3 rad/s 下 12~21° 相位滞后），旋转中不可用于校准
 *
 * 真车部署时本逻辑对应电控 MCU 固件（1kHz 陀螺积分 + 上位机慢速校准），
 * 仿真侧由 chassis_cmd_node 驱动，保持仿真/真车同架构。
 */

#include <cmath>

namespace sentry_planner
{

class ChassisYawEstimator
{
public:
    struct Config
    {
        double gate_wz = 0.5;        // |wz| 低于此值才接受 LIO 校准 (rad/s)
        double correction_gain = 1.0; // 校准增益 k (1/s)：err 以时间常数 1/k 收敛
        double max_imu_dt = 0.1;      // IMU 时间戳跳变超过此值丢弃该次积分 (s)
    };

    ChassisYawEstimator() = default;
    explicit ChassisYawEstimator(const Config &cfg) : cfg_(cfg) {}

    /** 陀螺仪样本：按时间戳积分。返回积分是否被采纳。 */
    bool onGyro(double stamp, double wz)
    {
        if (last_imu_stamp_ >= 0.0)
        {
            double dt = stamp - last_imu_stamp_;
            if (dt <= 0.0 || dt > cfg_.max_imu_dt)
            {
                last_imu_stamp_ = stamp;  // 时间戳异常：跳过本次，重新锚定
                return false;
            }
            if (initialized_)
                yaw_ = wrap(yaw_ + wz * dt);
            last_wz_ = wz;
        }
        last_imu_stamp_ = stamp;
        return true;
    }

    /** LIO yaw 观测：首次直接初始化；此后仅在低转速时做慢速校准。 */
    void onLioYaw(double yaw_lio)
    {
        if (!initialized_)
        {
            yaw_ = wrap(yaw_lio);
            initialized_ = true;
            return;
        }
        if (std::fabs(last_wz_) < cfg_.gate_wz && last_imu_stamp_ >= 0.0)
        {
            // 以 LIO 周期 (~20Hz) 施加一阶校准；dt 用固定标称值即可，
            // 增益语义是时间常数而非精确融合权重
            constexpr double lio_dt = 0.05;
            double err = wrap(yaw_lio - yaw_);
            yaw_ = wrap(yaw_ + cfg_.correction_gain * lio_dt * err);
        }
    }

    bool initialized() const { return initialized_; }
    double yaw() const { return yaw_; }
    double lastWz() const { return last_wz_; }

    static double wrap(double a)
    {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

private:
    Config cfg_{};
    double yaw_ = 0.0;
    double last_wz_ = 0.0;
    double last_imu_stamp_ = -1.0;
    bool initialized_ = false;
};

} // namespace sentry_planner
