#pragma once
/**
 * PlannerFsm: 局部规划器执行状态机（纯函数，无 ROS 依赖）
 *
 * 状态语义:
 *   IDLE     — 无有效轨迹（无目标 / 已到达 / 轨迹执行完）
 *   EXEC     — 正常跟踪轨迹
 *   SLOWDOWN — 前瞻窗口内轨迹不安全，降速执行近端安全段，等待重规划
 *   BRAKE    — 碰撞迫近或修复超时，立即刹停，丢弃当前轨迹
 *
 * 降级阶梯: EXEC → SLOWDOWN → BRAKE，任何状态由 PLAN_SUCCESS 拉回 EXEC。
 * 关键设计: PLAN_FAIL 不改变状态 —— 旧轨迹只要仍被安全监控认可就继续执行，
 * 周期性重规划失败不应丢弃一条还能用的轨迹。
 */

namespace sentry_planner
{

enum class FsmState
{
    IDLE,
    EXEC,
    SLOWDOWN,
    BRAKE,
};

enum class FsmEvent
{
    PLAN_SUCCESS,          // 新轨迹生成并交换成功
    PLAN_FAIL,             // 本轮重规划失败（A* 无路 / 采样不足）
    TRAJ_UNSAFE,           // 前瞻窗口内发现不安全点（尚有减速余量）
    TRAJ_UNSAFE_IMMINENT,  // 不安全点过近，减速已来不及，必须刹停
    TRAJ_SAFE,             // 此前不安全的轨迹重新变为安全（动障移开/地图更新）
    UNSAFE_TIMEOUT,        // SLOWDOWN 持续超时仍未修复
    TRAJ_FINISHED,         // 轨迹时间执行完毕
    GOAL_REACHED,          // 到达最终目标
    BRAKE_SETTLED,         // 刹停完成（静置且速度≈0）：降级 IDLE 重新找路
};

/** 纯状态转移函数：不产生副作用，动作由调用方依据 (旧状态, 新状态) 执行 */
constexpr FsmState fsmTransition(FsmState s, FsmEvent e)
{
    switch (e)
    {
    case FsmEvent::GOAL_REACHED:
        return FsmState::IDLE;

    case FsmEvent::PLAN_SUCCESS:
        return FsmState::EXEC;

    case FsmEvent::PLAN_FAIL:
        return s;  // 保留旧轨迹/旧状态，由安全监控决定是否降级

    case FsmEvent::TRAJ_FINISHED:
        return (s == FsmState::BRAKE) ? FsmState::BRAKE : FsmState::IDLE;

    case FsmEvent::TRAJ_UNSAFE:
        return (s == FsmState::EXEC) ? FsmState::SLOWDOWN : s;

    case FsmEvent::TRAJ_UNSAFE_IMMINENT:
        return (s == FsmState::EXEC || s == FsmState::SLOWDOWN) ? FsmState::BRAKE : s;

    case FsmEvent::TRAJ_SAFE:
        return (s == FsmState::SLOWDOWN) ? FsmState::EXEC : s;

    case FsmEvent::UNSAFE_TIMEOUT:
        return (s == FsmState::SLOWDOWN) ? FsmState::BRAKE : s;

    case FsmEvent::BRAKE_SETTLED:
        // BRAKE 的失败出口: 原先只有 PLAN_SUCCESS/GOAL_REACHED 可退出，
        // 规划连败即永久死锁（台沿实证 762 次失败 840s 冻结），且挂在
        // IDLE 分支的兜底行为（窄道爬行）永不可达
        return (s == FsmState::BRAKE) ? FsmState::IDLE : s;
    }
    return s;
}

constexpr const char *fsmStateName(FsmState s)
{
    switch (s)
    {
    case FsmState::IDLE:     return "IDLE";
    case FsmState::EXEC:     return "EXEC";
    case FsmState::SLOWDOWN: return "SLOWDOWN";
    case FsmState::BRAKE:    return "BRAKE";
    }
    return "?";
}

} // namespace sentry_planner
