#pragma once
/**
 * TrajectoryTracker: MPC 轨迹跟踪 + 窄道/坡道对齐模式
 *
 * 纯算法类，不依赖 ROS。输入当前状态和轨迹，输出 cmd_vel。
 *
 * 两种工作模式:
 *   - 常规: MPC 跟踪 MINCO 参考 + 自转 (spin_rate) 改善 LiDAR 覆盖。
 *   - 对齐 (窄道/坡道): 停自转，航向对齐轨迹方向 (车头或车尾，不侧行)，
 *     纯追踪恒低速直穿。纯追踪按几何弧长取前瞻点，不依赖时间参考 ——
 *     重规划把参考速度打回起步段时恒速指令不受影响 (常规 MPC 会)。
 *
 * 进入/退出判定 (带迟滞，防边界振荡):
 *   - 窄道: 轨迹前方 preview_time 内，任意采样点**两侧**侧向 side_probe 处
 *     ESDF 都 < corridor_enter_dist 才进入；都 > corridor_exit_dist 退出。
 *     双侧判定：开阔地里贴墙/贴柱的路径（单侧近、另一侧开阔）不算窄道，
 *     正常 MPC+自转可以绕，不必停转对齐；只有两侧都被墙夹住的真窄道/夹口才对齐。
 *   - 坡道: 机身倾角 (体轴 z 与重力夹角) > slope_enter_rad 进入；
 *     < slope_exit_rad 退出。坡上自转有侧翻风险，且斜面横行轮胎易打滑。
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
        // --- 窄道/坡道对齐模式 ---
        double corridor_enter_dist = 0.55;   // 前瞻 min esdf < 此值 → 进入 (m)
        double corridor_exit_dist = 0.80;    // min esdf > 此值 → 退出 (迟滞)
        double corridor_preview_time = 1.2;  // 沿轨迹前瞻时长 (s)
        double corridor_side_probe = 0.30;   // 窄道判定侧向探测距离 (m)：轨迹法向两侧采样
        double corridor_speed = 0.8;         // 对齐后恒速直穿 (m/s)
        double corridor_align_speed = 0.2;   // 未对齐时爬行速度 (m/s)
        double corridor_align_tol = 0.35;    // 航向误差 < 此值视为已对齐 (rad)
        double corridor_lookahead = 0.6;     // 纯追踪前瞻弧长 (m)
        double slope_enter_rad = 0.122;      // 倾角 > 此值 → 坡道模式 (~7°)
        double slope_exit_rad = 0.070;       // 倾角 < 此值退出 (~4°)
        double yaw_align_kp = 3.0;
        double max_align_wz = 2.5;           // 对齐角速度上限 (rad/s)
        double min_turn_radius = 0.6;        // 对齐模式最小转弯半径 (m)：R = v/|wz| 下限，
                                             // 防止小半径转向在坡脚/台阶沿把轮子别住
        double wz_slew_rate = 8.0;           // 输出角速度限速率 (rad/s²)：自转↔对齐切换平滑
        int mpc_horizon = 10;
        double mpc_q_pos = 10.0;
        double mpc_q_vel = 1.0;
        double mpc_r_acc = 0.1;
        // true: cmd.linear 保持 odom/世界系，由下游（chassis_cmd_node / 电控 MCU）
        // 用高频陀螺 yaw 旋转到机体系 —— 避免用带龄期的 LIO yaw 在 50Hz 侧旋转
        bool world_frame_cmd = false;
    };

    void init(const Config &cfg, sentry_nav::EnvironmentInterface *env);

    /** 参数热重载: 运行时替换配置并同步 MPC 内部配置。
     *  并发说明: 50Hz 控制线程在读 cfg_，字段为对齐 double 单字段写入原子，
     *  跨字段撕裂最多影响一拍——调参场景可接受。 */
    void updateConfig(const Config &cfg);

    /**
     * 计算 cmd_vel
     * @param pos       当前位置 (odom 系)
     * @param vel       当前速度 (odom 系)
     * @param yaw       当前航向
     * @param tilt      机身倾角 (体轴 z 与世界 z 夹角, rad)
     * @param traj      MINCO 轨迹
     * @param elapsed   轨迹已过时间
     * @param has_traj  是否有有效轨迹
     * @return          cmd_vel (world_frame_cmd 时 linear 为 odom 系)
     */
    geometry_msgs::msg::Twist compute(
        const Eigen::Vector2d &pos, const Eigen::Vector2d &vel, double yaw, double tilt,
        const MincoTrajectory &traj, double elapsed, bool has_traj);

    /** 当前是否处于窄道/坡道对齐模式（供节点侧决定 SLOWDOWN 缩放等） */
    bool inAlignMode() const { return narrow_on_ || slope_on_; }
    bool inNarrowMode() const { return narrow_on_; }
    bool inSlopeMode() const { return slope_on_; }

    /**
     * 窄道直穿兜底指令: 无轨迹时朝给定目标点爬行（对齐折向 + 恒速 + 脊线纠偏）。
     * 供节点在"规划失败但静态验证的全局路径仍在"时沿 JPS 路径硬穿窄口 —
     * 活图噪声让 A* footprint 无路时，规划管线出不来轨迹，兜底不依赖它。
     */
    geometry_msgs::msg::Twist computeCrawl(
        const Eigen::Vector2d &pos, double yaw, Eigen::Vector2d target);

private:
    /** 迟滞状态机: 更新 narrow_on_ / slope_on_ */
    void updateMode(const Eigen::Vector2d &pos, double tilt,
                    const MincoTrajectory &traj, double elapsed);

    /** 对齐模式指令: 纯追踪方向 + 恒速 + 航向对齐 (折到车头/车尾)。
     *  计算世界系线速度 v_odom 与角速度(返回)。帧旋转与限速由 compute() 统一处理。 */
    double computeAlignCmd(
        const Eigen::Vector2d &pos, double yaw,
        const MincoTrajectory &traj, double elapsed,
        Eigen::Vector2d &v_odom);

    /** 目标点沿 ESDF 梯度推向脊线 (缝中心线)。仅窄道有意义; 坡上禁用 (见 computeAlignCmd)。 */
    void nudgeToRidge(Eigen::Vector2d &p) const;

    /** 朝 target 方向: 计算世界系线速度 v_odom 与对齐角速度 wz (折到车头/车尾)。
     *  返回 false = 已贴近目标 (停车)。帧旋转由调用方处理。 */
    bool alignCmdToward(
        const Eigen::Vector2d &pos, double yaw, const Eigen::Vector2d &target,
        Eigen::Vector2d &v_odom, double &wz) const;

    Config cfg_;
    MPCController mpc_;
    sentry_nav::EnvironmentInterface *env_ = nullptr;
    bool narrow_on_ = false;
    bool slope_on_ = false;
    double last_wz_cmd_ = 0.0;  // 角速度限速器状态 (rad/s)
    Eigen::Vector2d last_lin_cmd_ = Eigen::Vector2d::Zero();  // 线速度限速器状态 (odom 系, m/s)
};

} // namespace sentry_planner
