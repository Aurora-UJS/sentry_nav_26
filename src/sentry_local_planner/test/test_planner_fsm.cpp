/**
 * PlannerFsm 状态转移表全覆盖测试
 */
#include <gtest/gtest.h>
#include <sentry_local_planner/planner_fsm.hpp>

using sentry_planner::FsmEvent;
using sentry_planner::FsmState;
using sentry_planner::fsmTransition;

TEST(PlannerFsm, PlanSuccessAlwaysEntersExec)
{
    for (auto s : {FsmState::IDLE, FsmState::EXEC, FsmState::SLOWDOWN, FsmState::BRAKE})
        EXPECT_EQ(fsmTransition(s, FsmEvent::PLAN_SUCCESS), FsmState::EXEC);
}

TEST(PlannerFsm, GoalReachedAlwaysReturnsIdle)
{
    for (auto s : {FsmState::IDLE, FsmState::EXEC, FsmState::SLOWDOWN, FsmState::BRAKE})
        EXPECT_EQ(fsmTransition(s, FsmEvent::GOAL_REACHED), FsmState::IDLE);
}

// 核心设计：周期性重规划失败不改变状态 —— 不丢弃仍然安全的旧轨迹
TEST(PlannerFsm, PlanFailKeepsState)
{
    for (auto s : {FsmState::IDLE, FsmState::EXEC, FsmState::SLOWDOWN, FsmState::BRAKE})
        EXPECT_EQ(fsmTransition(s, FsmEvent::PLAN_FAIL), s);
}

TEST(PlannerFsm, DegradationLadder)
{
    // EXEC → SLOWDOWN → BRAKE
    EXPECT_EQ(fsmTransition(FsmState::EXEC, FsmEvent::TRAJ_UNSAFE), FsmState::SLOWDOWN);
    EXPECT_EQ(fsmTransition(FsmState::SLOWDOWN, FsmEvent::UNSAFE_TIMEOUT), FsmState::BRAKE);
    // 碰撞迫近跳过降速直接刹停
    EXPECT_EQ(fsmTransition(FsmState::EXEC, FsmEvent::TRAJ_UNSAFE_IMMINENT), FsmState::BRAKE);
    EXPECT_EQ(fsmTransition(FsmState::SLOWDOWN, FsmEvent::TRAJ_UNSAFE_IMMINENT), FsmState::BRAKE);
}

TEST(PlannerFsm, SlowdownRecoversWhenTrajSafeAgain)
{
    EXPECT_EQ(fsmTransition(FsmState::SLOWDOWN, FsmEvent::TRAJ_SAFE), FsmState::EXEC);
    // TRAJ_SAFE 对其他状态无效
    EXPECT_EQ(fsmTransition(FsmState::EXEC, FsmEvent::TRAJ_SAFE), FsmState::EXEC);
    EXPECT_EQ(fsmTransition(FsmState::BRAKE, FsmEvent::TRAJ_SAFE), FsmState::BRAKE);
    EXPECT_EQ(fsmTransition(FsmState::IDLE, FsmEvent::TRAJ_SAFE), FsmState::IDLE);
}

TEST(PlannerFsm, TrajFinished)
{
    EXPECT_EQ(fsmTransition(FsmState::EXEC, FsmEvent::TRAJ_FINISHED), FsmState::IDLE);
    EXPECT_EQ(fsmTransition(FsmState::SLOWDOWN, FsmEvent::TRAJ_FINISHED), FsmState::IDLE);
    // BRAKE 只能由新轨迹或到达目标解除，轨迹计时结束不能悄悄放开刹车
    EXPECT_EQ(fsmTransition(FsmState::BRAKE, FsmEvent::TRAJ_FINISHED), FsmState::BRAKE);
}

TEST(PlannerFsm, UnsafeEventsIgnoredOutsideExecution)
{
    EXPECT_EQ(fsmTransition(FsmState::IDLE, FsmEvent::TRAJ_UNSAFE), FsmState::IDLE);
    EXPECT_EQ(fsmTransition(FsmState::IDLE, FsmEvent::TRAJ_UNSAFE_IMMINENT), FsmState::IDLE);
    EXPECT_EQ(fsmTransition(FsmState::BRAKE, FsmEvent::TRAJ_UNSAFE), FsmState::BRAKE);
    EXPECT_EQ(fsmTransition(FsmState::EXEC, FsmEvent::UNSAFE_TIMEOUT), FsmState::EXEC);
    // 已在降速中再次报 TRAJ_UNSAFE 保持降速
    EXPECT_EQ(fsmTransition(FsmState::SLOWDOWN, FsmEvent::TRAJ_UNSAFE), FsmState::SLOWDOWN);
}
