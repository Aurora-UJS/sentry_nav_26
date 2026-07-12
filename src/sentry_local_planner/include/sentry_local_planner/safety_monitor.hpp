#pragma once
/**
 * SafetyMonitor: 执行中轨迹前瞻安全检查（纯函数，无 ROS 依赖）
 *
 * 以固定时间步采样 [t_from, t_to] 内的轨迹位置，逐点查询 ESDF。
 * 每次查询 O(1)，2s 前瞻 / 0.03s 步长 ≈ 67 次查询，放在 50Hz 控制回路里可忽略。
 *
 * 阈值分两级（必须与规划器的验收标准拉开距离，否则会振荡）:
 *   - margin_warn: 比规划器有意生成的贴墙间隙更低才告警。MINCO 以 footprint 点
 *     esdf >= dist0(0.05) 验收，贴墙轨迹中心 esdf 合法地低到 ~0.28m；监控若在
 *     robot_radius(0.3) 一刀切，会把规划器的正常输出判为不安全 → EXEC/BRAKE 振荡
 *     （实测 RMUC 仿真 204 次/2min）。
 *   - margin_hard: 轨迹真正穿入障碍（新障碍挡路时采样点 esdf → 0 必然穿过此值）。
 *
 * 注意: 采样点出地图按硬不安全处理 —— 滑窗地图外的区域不可轻信
 * （rose 与本项目旧逻辑都把 out-of-map 当安全，这里明确修掉）。
 *
 * self_ignore_radius: 距机器人当前位置过近的采样点不参与判定。机器人已经
 * 占据的区域不构成"未来碰撞"；贴墙脱困时规划器输出的近端必然贴障，若不豁免
 * 会出现 PLAN_SUCCESS → 同拍被硬检查打回 BRAKE 的死循环（实测 RMUC 仿真）。
 */

#include <Eigen/Eigen>
#include <plan_env/environment_interface.hpp>
#include <sentry_local_planner/minco_trajectory.hpp>

namespace sentry_planner
{

struct SafetyCheckResult
{
    double min_distance = 1e9;     // 检查窗口内最小 ESDF 距离
    double first_warn_time = -1.0; // 首个 esdf < margin_warn 的轨迹时刻；无则 -1
    double first_hard_time = -1.0; // 首个 esdf < margin_hard 的轨迹时刻；无则 -1

    bool warn() const { return first_warn_time >= 0.0; }
    bool hard() const { return first_hard_time >= 0.0; }
};

/**
 * @param env          ESDF 环境（O(1) 距离查询）
 * @param traj         当前执行的轨迹
 * @param t_from       检查起点（通常 = elapsed）
 * @param t_to         检查终点（通常 = min(elapsed + horizon, duration)）
 * @param dt           时间步长；应满足 max_vel * dt ≲ 2 * voxel_size，防止穿墙漏检
 * @param margin_warn  软阈值：低于它进入降速
 * @param margin_hard  硬阈值：低于它视为真碰撞路径（须 < margin_warn）
 * @param current_pos  机器人当前位置
 * @param self_ignore_radius  距当前位置小于此值的采样点跳过（脱困豁免）
 *
 * 全窗口扫描（不提前返回），min_distance 供退出迟滞判断使用。
 */
inline SafetyCheckResult checkTrajectorySafety(
    sentry_nav::EnvironmentInterface &env, const MincoTrajectory &traj,
    double t_from, double t_to, double dt,
    double margin_warn, double margin_hard,
    const Eigen::Vector2d &current_pos, double self_ignore_radius)
{
    SafetyCheckResult res;
    if (traj.empty() || t_to <= t_from || dt <= 0.0)
        return res;

    for (double t = t_from; t <= t_to; t += dt)
    {
        Eigen::Vector2d p = traj.getPosition(t);

        if ((p - current_pos).norm() < self_ignore_radius)
            continue;  // 机器人已占据的区域不算未来碰撞

        double d;
        if (!env.isInMap(p))
            d = 0.0;  // 出地图视为硬不安全
        else
            d = env.getDistance(p);

        if (d < res.min_distance)
            res.min_distance = d;

        if (d < margin_warn && res.first_warn_time < 0.0)
            res.first_warn_time = t;
        if (d < margin_hard && res.first_hard_time < 0.0)
            res.first_hard_time = t;
    }
    return res;
}

} // namespace sentry_planner
