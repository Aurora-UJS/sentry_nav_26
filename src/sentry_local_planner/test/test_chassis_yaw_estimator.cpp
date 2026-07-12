/**
 * ChassisYawEstimator 测试：陀螺积分 + 门控互补校准
 */
#include <gtest/gtest.h>
#include <cmath>
#include <sentry_local_planner/chassis_yaw_estimator.hpp>

using sentry_planner::ChassisYawEstimator;

TEST(ChassisYawEstimator, InitFromFirstLioYaw)
{
    ChassisYawEstimator est;
    EXPECT_FALSE(est.initialized());
    est.onLioYaw(1.2);
    ASSERT_TRUE(est.initialized());
    EXPECT_DOUBLE_EQ(est.yaw(), 1.2);
}

TEST(ChassisYawEstimator, GyroIntegration)
{
    ChassisYawEstimator est;
    est.onLioYaw(0.0);
    double t = 0.0;
    for (int i = 0; i < 100; ++i)  // 1s @ 100Hz, wz=2 → yaw=2
    {
        t += 0.01;
        est.onGyro(t, 2.0);
    }
    EXPECT_NEAR(est.yaw(), 2.0, 0.03);
}

TEST(ChassisYawEstimator, YawWrapsAtPi)
{
    ChassisYawEstimator est;
    est.onLioYaw(3.0);
    double t = 0.0;
    for (int i = 0; i < 50; ++i)  // 0.5s, wz=1 → 3.5 → wrap to 3.5-2π
    {
        t += 0.01;
        est.onGyro(t, 1.0);
    }
    EXPECT_NEAR(est.yaw(), 3.5 - 2.0 * M_PI, 0.03);
}

// 核心门控语义：高速自转时 LIO 校准被拒绝（自转下 LIO yaw 有龄期滞后不可信）
TEST(ChassisYawEstimator, LioCorrectionGatedDuringSpin)
{
    ChassisYawEstimator::Config cfg;
    cfg.gate_wz = 0.5;
    ChassisYawEstimator est(cfg);
    est.onLioYaw(0.0);

    // 自转中 (wz=3 > gate): 注入带 -0.3 rad 滞后误差的 LIO yaw，不应被采纳
    double t = 0.0;
    for (int i = 0; i < 100; ++i)
    {
        t += 0.01;
        est.onGyro(t, 3.0);
        est.onLioYaw(ChassisYawEstimator::wrap(est.yaw() - 0.3));  // 伪装滞后观测
    }
    // 若校准被门控，yaw 应≈纯积分值 3.0 (wrap 后 3-2π)
    EXPECT_NEAR(est.yaw(), ChassisYawEstimator::wrap(3.0), 0.05);
}

// 停转后校准开闸：常值偏差按增益时间常数收敛
TEST(ChassisYawEstimator, LioCorrectionConvergesWhenSlow)
{
    ChassisYawEstimator::Config cfg;
    cfg.gate_wz = 0.5;
    cfg.correction_gain = 1.0;
    ChassisYawEstimator est(cfg);
    est.onLioYaw(0.0);

    double t = 0.0;
    est.onGyro(t += 0.01, 0.0);  // 静止
    // 注入常值 0.2 rad 偏差的 LIO 观测，20Hz × 5s，k=1 → 应基本收敛
    for (int i = 0; i < 100; ++i)
    {
        est.onGyro(t += 0.05, 0.0);
        est.onLioYaw(0.2);
    }
    EXPECT_NEAR(est.yaw(), 0.2, 0.02);
}

TEST(ChassisYawEstimator, BadImuStampRejected)
{
    ChassisYawEstimator est;
    est.onLioYaw(0.0);
    EXPECT_TRUE(est.onGyro(1.00, 1.0));   // 锚定
    EXPECT_TRUE(est.onGyro(1.01, 1.0));   // 正常积分
    EXPECT_FALSE(est.onGyro(1.005, 1.0)); // 时间倒退 → 拒绝
    EXPECT_FALSE(est.onGyro(2.0, 1.0));   // 跳变 > max_imu_dt → 拒绝并重新锚定
    EXPECT_TRUE(est.onGyro(2.01, 1.0));   // 恢复
    // 只有两次 0.01s 的有效积分
    EXPECT_NEAR(est.yaw(), 0.02, 1e-9);
}
