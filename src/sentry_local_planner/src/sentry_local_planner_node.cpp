/**
 * sentry_local_planner_node: Kinodynamic A* + MINCO + MPC
 *
 * 执行安全架构（三个时序域解耦）:
 *   - 50Hz 控制回路: 跟踪 + 执行中轨迹前瞻安全监控（safety_monitor.hpp）
 *   - 10Hz 重规划: 全量 A* + MINCO，规划失败不丢弃仍安全的旧轨迹
 *   - 状态机: IDLE/EXEC/SLOWDOWN/BRAKE 纯函数转移（planner_fsm.hpp），
 *     降级阶梯 EXEC → SLOWDOWN → BRAKE 保证阶梯有底（刹停），不裸奔
 *
 * 轨迹以 shared_ptr<const TrajSnapshot> 整体快照交换（轨迹+起始时刻+时长），
 * 控制回路每拍取一次快照，杜绝规划中途改写轨迹导致的撕裂读取。
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <mutex>

#include <plan_env/sdf_map.hpp>
#include <plan_env/edt_environment.hpp>
#include <path_searching/kinodynamic_astar.hpp>

#include <sentry_local_planner/minco_trajectory.hpp>
#include <sentry_local_planner/trajectory_tracker.hpp>
#include <sentry_local_planner/planner_fsm.hpp>
#include <sentry_local_planner/safety_monitor.hpp>

using namespace fast_planner;
using sentry_planner::FsmEvent;
using sentry_planner::FsmState;

class SentryLocalPlannerNode : public rclcpp::Node
{
public:
    SentryLocalPlannerNode() : Node("sentry_local_planner")
    {
        this->declare_parameter<std::string>("manager.odometry", "/Odometry");
    }

    void initialize()
    {
        auto node_ptr = this->shared_from_this();

        // --- Init plan_env ---
        sdf_map_ = std::make_shared<SDFMap>();
        sdf_map_->initMap(node_ptr);
        edt_env_ = std::make_shared<EDTEnvironment>();
        edt_env_->setMap(sdf_map_);

        // --- Init A* ---
        kino_astar_ = std::make_shared<KinodynamicAstar>();
        kino_astar_->setParam(node_ptr);
        kino_astar_->setEnvironment(edt_env_.get());
        kino_astar_->init();

        // --- Init MINCO ---
        this->declare_parameter<double>("minco_opt.lambda_smooth", 0.1);
        this->declare_parameter<double>("minco_opt.lambda_col",    8.0);
        this->declare_parameter<double>("minco_opt.lambda_feas",   0.001);
        this->declare_parameter<double>("minco_opt.dist0",         0.05);
        this->declare_parameter<double>("minco_opt.robot_radius",  0.3);
        this->declare_parameter<int>   ("minco_opt.num_samples",   8);
        this->declare_parameter<int>   ("minco_opt.max_iter",      200);
        this->declare_parameter<double>("minco_opt.max_time_ms",   20.0);

        double mv = 3.0, ma = 3.0;
        this->get_parameter("search.max_vel", mv);
        this->get_parameter("search.max_acc", ma);
        minco_traj_.setOptimizer(
            edt_env_.get(),
            this->get_parameter("minco_opt.lambda_smooth").as_double(),
            this->get_parameter("minco_opt.lambda_col").as_double(),
            this->get_parameter("minco_opt.lambda_feas").as_double(),
            this->get_parameter("minco_opt.dist0").as_double(),
            mv, ma,
            this->get_parameter("minco_opt.robot_radius").as_double(),
            this->get_parameter("minco_opt.num_samples").as_int(),
            this->get_parameter("minco_opt.max_iter").as_int(),
            this->get_parameter("minco_opt.max_time_ms").as_double() / 1000.0);

        // --- Init TrajectoryTracker ---
        this->declare_parameter<double>("controller.frequency", 50.0);
        this->declare_parameter<double>("controller.spin_rate", 3.0);
        this->declare_parameter<double>("controller.narrow_passage_dist", 0.5);
        this->declare_parameter<double>("controller.yaw_align_kp", 3.0);
        this->declare_parameter<int>("mpc.horizon", 10);
        this->declare_parameter<double>("mpc.q_pos", 10.0);
        this->declare_parameter<double>("mpc.q_vel", 1.0);
        this->declare_parameter<double>("mpc.r_acc", 0.1);

        double ctrl_freq = this->get_parameter("controller.frequency").as_double();

        sentry_planner::TrajectoryTracker::Config tcfg;
        tcfg.max_vel = mv;
        tcfg.max_acc = ma;
        tcfg.ctrl_dt = 1.0 / ctrl_freq;
        tcfg.spin_rate = this->get_parameter("controller.spin_rate").as_double();
        tcfg.narrow_passage_dist = this->get_parameter("controller.narrow_passage_dist").as_double();
        tcfg.yaw_align_kp = this->get_parameter("controller.yaw_align_kp").as_double();
        tcfg.mpc_horizon = this->get_parameter("mpc.horizon").as_int();
        tcfg.mpc_q_pos = this->get_parameter("mpc.q_pos").as_double();
        tcfg.mpc_q_vel = this->get_parameter("mpc.q_vel").as_double();
        tcfg.mpc_r_acc = this->get_parameter("mpc.r_acc").as_double();
        // world: 输出 odom 系速度到 /cmd_vel_world，由 chassis_cmd_node（真车=电控）
        // 用高频陀螺 yaw 旋转；body: 旧行为，本节点用 LIO yaw 旋转后直发 /cmd_vel
        this->declare_parameter<std::string>("controller.cmd_frame", "world");
        std::string cmd_frame = this->get_parameter("controller.cmd_frame").as_string();
        tcfg.world_frame_cmd = (cmd_frame == "world");
        tracker_.init(tcfg, edt_env_.get());

        // --- 执行安全监控参数 ---
        this->declare_parameter<double>("safety.check_horizon", 2.0);
        this->declare_parameter<double>("safety.check_dt", 0.03);
        this->declare_parameter<double>("safety.margin_warn", 0.20);
        this->declare_parameter<double>("safety.margin_hard", 0.15);
        this->declare_parameter<double>("safety.exit_hysteresis", 0.05);
        this->declare_parameter<double>("safety.self_ignore_radius", 0.35);
        this->declare_parameter<double>("safety.imminent_time", 0.8);
        this->declare_parameter<double>("safety.slowdown_factor", 0.4);
        this->declare_parameter<double>("safety.brake_timeout", 0.5);
        safety_check_horizon_ = this->get_parameter("safety.check_horizon").as_double();
        safety_check_dt_      = this->get_parameter("safety.check_dt").as_double();
        safety_margin_warn_   = this->get_parameter("safety.margin_warn").as_double();
        safety_margin_hard_   = this->get_parameter("safety.margin_hard").as_double();
        safety_exit_hyst_     = this->get_parameter("safety.exit_hysteresis").as_double();
        safety_self_ignore_r_ = this->get_parameter("safety.self_ignore_radius").as_double();
        safety_imminent_time_ = this->get_parameter("safety.imminent_time").as_double();
        slowdown_factor_      = this->get_parameter("safety.slowdown_factor").as_double();
        brake_timeout_        = this->get_parameter("safety.brake_timeout").as_double();

        // --- 按需重规划参数 ---
        this->declare_parameter<double>("replan.deviation", 0.5);
        this->declare_parameter<double>("replan.refresh_period", 1.0);
        this->declare_parameter<double>("replan.plan_lead", 0.15);
        replan_deviation_      = this->get_parameter("replan.deviation").as_double();
        replan_refresh_period_ = this->get_parameter("replan.refresh_period").as_double();
        replan_plan_lead_      = this->get_parameter("replan.plan_lead").as_double();

        // --- Subscribers ---
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                current_pos_(0) = msg->pose.pose.position.x;
                current_pos_(1) = msg->pose.pose.position.y;
                double qw = msg->pose.pose.orientation.w;
                double qz = msg->pose.pose.orientation.z;
                current_yaw_ = 2.0 * atan2(qz, qw);

                // 速度反馈：优先用 ESKF twist（REP 105: base_link 系，旋转到 odom 系），
                // 供 A* 起点速度与 MPC 状态反馈使用。twist 恒为零（旧版 LIO 未填）
                // 时退回位姿差分 + 低通。
                Eigen::Vector2d v_body(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
                double t = rclcpp::Time(msg->header.stamp).seconds();
                if (v_body.squaredNorm() > 1e-12 || twist_seen_)
                {
                    twist_seen_ = true;
                    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                         msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
                    Eigen::Vector3d v_odom = q * Eigen::Vector3d(v_body(0), v_body(1),
                                                                 msg->twist.twist.linear.z);
                    current_vel_ = v_odom.head<2>();
                }
                else if (last_odom_time_ > 0.0)
                {
                    double dt = t - last_odom_time_;
                    if (dt > 1e-4 && dt < 0.5)
                    {
                        Eigen::Vector2d v_raw = (current_pos_ - last_odom_pos_) / dt;
                        current_vel_ = 0.6 * current_vel_ + 0.4 * v_raw;
                    }
                }
                last_odom_pos_ = current_pos_;
                last_odom_time_ = t;
                has_odom_ = true;
            });

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&SentryLocalPlannerNode::goalCallback, this, std::placeholders::_1));

        global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/global_path", 10,
            [this](const nav_msgs::msg::Path::SharedPtr msg) {
                global_waypoints_.clear();
                for (auto &pose : msg->poses)
                    global_waypoints_.emplace_back(pose.pose.position.x, pose.pose.position.y);
                current_waypoint_idx_ = 0;
                has_global_path_ = !global_waypoints_.empty();
                RCLCPP_INFO(this->get_logger(), "Received global path: %zu waypoints",
                            global_waypoints_.size());
            });

        // --- Publishers ---
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/trajectory", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/planning/trajectory_markers", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            tcfg.world_frame_cmd ? "/cmd_vel_world" : "/cmd_vel", 10);

        // --- Timers ---
        ctrl_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / ctrl_freq),
            std::bind(&SentryLocalPlannerNode::controlLoop, this));

        this->declare_parameter<double>("replan.frequency", 10.0);
        double replan_freq = this->get_parameter("replan.frequency").as_double();
        replan_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / replan_freq),
            std::bind(&SentryLocalPlannerNode::replanCallback, this));

        RCLCPP_INFO(this->get_logger(),
                    "Planner initialized: minco+mpc, ctrl@%.0fHz, safety horizon %.1fs",
                    ctrl_freq, safety_check_horizon_);
    }

private:
    /** 轨迹快照：轨迹 + 起始时刻 + 时长整体交换，避免三者被撕裂读取 */
    struct TrajSnapshot
    {
        sentry_planner::MincoTrajectory traj;
        rclcpp::Time start_time;
        double duration = 0.0;
    };
    using TrajSnapshotPtr = std::shared_ptr<const TrajSnapshot>;

    TrajSnapshotPtr getActiveTraj()
    {
        std::lock_guard<std::mutex> lk(traj_mutex_);
        return active_traj_;
    }
    void setActiveTraj(TrajSnapshotPtr snap)
    {
        std::lock_guard<std::mutex> lk(traj_mutex_);
        active_traj_ = std::move(snap);
    }

    enum class PlanResult { SUCCESS, FAILED, SKIPPED };

    /** 状态机事件分发：转移由纯函数决定，进入新状态的副作用集中在这里 */
    void dispatch(FsmEvent e)
    {
        FsmState next = sentry_planner::fsmTransition(state_, e);
        if (next == state_)
            return;

        RCLCPP_INFO(this->get_logger(), "FSM: %s -> %s",
                    sentry_planner::fsmStateName(state_),
                    sentry_planner::fsmStateName(next));

        if (next == FsmState::SLOWDOWN && state_ == FsmState::EXEC)
            unsafe_since_ = this->now();

        if (next == FsmState::BRAKE)
        {
            // 刹停即放弃当前轨迹；后续恢复只能靠新的 PLAN_SUCCESS
            setActiveTraj(nullptr);
            RCLCPP_WARN(this->get_logger(), "Trajectory dropped, braking");
        }

        if (next == FsmState::IDLE)
            setActiveTraj(nullptr);

        state_ = next;
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!has_odom_) { RCLCPP_WARN(this->get_logger(), "No odom yet"); return; }
        final_goal_ = Eigen::Vector2d(msg->pose.position.x, msg->pose.position.y);
        has_final_goal_ = true;
        dispatchPlanResult(planToGoal(final_goal_));
    }

    void dispatchPlanResult(PlanResult res)
    {
        if (res == PlanResult::SUCCESS)
            dispatch(FsmEvent::PLAN_SUCCESS);
        else if (res == PlanResult::FAILED)
        {
            dispatch(FsmEvent::PLAN_FAIL);
            if (state_ != FsmState::IDLE)
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Replan failed, state=%s",
                                     sentry_planner::fsmStateName(state_));
        }
    }

    PlanResult planToGoal(Eigen::Vector2d goal_pt)
    {
        Eigen::Vector2d diff = goal_pt - current_pos_;
        // A* near_end tolerance = ceil(1/resolution_astar) * resolution_astar ≈ 1.0m
        // 不要在这个范围内重规划，让最后一段轨迹自然执行完
        if (diff.norm() < 1.0) return PlanResult::SKIPPED;

        double local_range = 7.5;
        this->get_parameter("sdf_map.local_update_range_x", local_range);
        local_range -= 0.5;
        if (diff.norm() > local_range)
            goal_pt = current_pos_ + diff.normalized() * local_range;

        Eigen::Vector2d map_origin, map_size;
        sdf_map_->getRegion(map_origin, map_size);
        for (int i = 0; i < 2; ++i)
            goal_pt(i) = std::clamp(goal_pt(i), map_origin(i) + 0.5, map_origin(i) + map_size(i) - 0.5);

        // 起点状态：执行中且跟踪良好时，从当前轨迹的参考点 (elapsed + lead) 续接，
        // 而不是从实测状态重启 —— 否则每次重规划都把参考速度打回起步段，
        // 10Hz 重规划下参考速度永远爬不上去（实测 5.4m 直线平均有效速度仅 0.27m/s）。
        // 跟踪偏差过大时回退到实测状态重锚（MPC 已跟不上参考，续接无意义）。
        Eigen::Vector2d start_pos = current_pos_;
        Eigen::Vector2d start_vel = current_vel_;
        Eigen::Vector2d start_acc = Eigen::Vector2d::Zero();
        rclcpp::Time traj_t0 = this->now();
        auto prev = getActiveTraj();
        if (prev && (state_ == FsmState::EXEC || state_ == FsmState::SLOWDOWN))
        {
            double elapsed = (this->now() - prev->start_time).seconds();
            double t_ref = std::min(elapsed + replan_plan_lead_, prev->duration);
            Eigen::Vector2d p_ref = prev->traj.getPosition(t_ref);
            if ((p_ref - current_pos_).norm() < replan_deviation_)
            {
                start_pos = p_ref;
                start_vel = prev->traj.getVelocity(t_ref);
                start_acc = prev->traj.getAcceleration(t_ref);
                traj_t0 = prev->start_time + rclcpp::Duration::from_seconds(t_ref);
            }
        }

        // A*
        kino_astar_->reset();
        int result = kino_astar_->search(start_pos, start_vel, start_acc,
                                          goal_pt, Eigen::Vector2d::Zero(), true);
        if (result == KinodynamicAstar::NO_PATH) return PlanResult::FAILED;

        double ts;
        std::vector<Eigen::Vector2d> point_set, start_end_derivatives;
        kino_astar_->getSamples(ts, point_set, start_end_derivatives);
        if (point_set.size() < 2) return PlanResult::FAILED;

        // MINCO
        double max_vel = 3.0, max_acc = 3.0;
        this->get_parameter("search.max_vel", max_vel);
        this->get_parameter("search.max_acc", max_acc);

        Eigen::Vector2d sv = start_end_derivatives.size() >= 2 ? start_end_derivatives[0] : start_vel;
        Eigen::Vector2d ev = start_end_derivatives.size() >= 2 ? start_end_derivatives[1] : Eigen::Vector2d::Zero();
        Eigen::Vector2d sa = start_end_derivatives.size() >= 4 ? start_end_derivatives[2] : start_acc;
        Eigen::Vector2d ea = start_end_derivatives.size() >= 4 ? start_end_derivatives[3] : Eigen::Vector2d::Zero();

        minco_traj_.setup(point_set, sv, sa, ev, ea, max_vel, max_acc, this->now().seconds());
        if (minco_traj_.empty()) return PlanResult::FAILED;

        auto snap = std::make_shared<TrajSnapshot>();
        snap->traj = minco_traj_;  // 值拷贝进只读快照，工作对象后续改写不影响控制回路
        snap->start_time = traj_t0;
        snap->duration = minco_traj_.getDuration();
        setActiveTraj(snap);

        last_plan_time_ = this->now();
        last_planned_goal_ = goal_pt;
        publishTrajectory(snap->traj, snap->duration);
        return PlanResult::SUCCESS;
    }

    void replanCallback()
    {
        if (!has_odom_ || !has_final_goal_) return;
        if ((final_goal_ - current_pos_).norm() < 1.0) return;

        Eigen::Vector2d local_goal = final_goal_;
        if (has_global_path_ && !global_waypoints_.empty())
        {
            while (current_waypoint_idx_ < (int)global_waypoints_.size() - 1 &&
                   (global_waypoints_[current_waypoint_idx_] - current_pos_).norm() < 1.0)
                current_waypoint_idx_++;
            local_goal = global_waypoints_[current_waypoint_idx_];
        }

        // 按需重规划，而不是每拍无条件全量重规划（借鉴 rose 的局部性思想）：
        // 正常执行中让轨迹跑完它的加速段，只在以下情况重新规划 —
        //   无轨迹(IDLE/BRAKE 有目标) / SLOWDOWN(找更好的路) / 跟踪偏差超限(重锚)
        //   / 目标点移动 / 周期性提质刷新
        auto snap = getActiveTraj();
        bool need_replan = false;
        if (!snap)
            need_replan = true;
        else if (state_ == FsmState::SLOWDOWN)
            need_replan = true;
        else
        {
            double elapsed = (this->now() - snap->start_time).seconds();
            double deviation =
                (snap->traj.getPosition(std::min(elapsed, snap->duration)) - current_pos_).norm();
            need_replan = deviation > replan_deviation_ ||
                          (local_goal - last_planned_goal_).norm() > 0.3 ||
                          (this->now() - last_plan_time_).seconds() > replan_refresh_period_;
        }

        if (need_replan)
            dispatchPlanResult(planToGoal(local_goal));
    }

    void controlLoop()
    {
        if (!has_odom_) return;

        auto snap = getActiveTraj();

        if (snap)
        {
            double elapsed = (this->now() - snap->start_time).seconds();

            if (elapsed >= snap->duration)
            {
                if (!has_final_goal_ || (final_goal_ - current_pos_).norm() < 1.0)
                {
                    RCLCPP_INFO(this->get_logger(), "Goal reached!");
                    has_final_goal_ = false;
                    dispatch(FsmEvent::GOAL_REACHED);
                }
                else
                    dispatch(FsmEvent::TRAJ_FINISHED);
                snap.reset();
            }
            else if (state_ == FsmState::EXEC || state_ == FsmState::SLOWDOWN)
            {
                // 前瞻安全监控：每拍 ~70 次 O(1) ESDF 查询。
                // 分级判定：hard（真穿障）才允许升级到 BRAKE；warn（比规划器
                // 有意的贴墙更近）只降速，避免与规划器验收标准打架产生振荡。
                double t_end = std::min(elapsed + safety_check_horizon_, snap->duration);
                auto chk = sentry_planner::checkTrajectorySafety(
                    *edt_env_, snap->traj, elapsed, t_end, safety_check_dt_,
                    safety_margin_warn_, safety_margin_hard_,
                    current_pos_, safety_self_ignore_r_);

                if (chk.hard())
                {
                    Eigen::Vector2d hp = snap->traj.getPosition(chk.first_hard_time);
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                         "Traj hits obstacle in %.2fs (min esdf %.2fm) at (%.2f, %.2f), robot (%.2f, %.2f)",
                                         chk.first_hard_time - elapsed, chk.min_distance,
                                         hp(0), hp(1), current_pos_(0), current_pos_(1));
                    if (chk.first_hard_time - elapsed < safety_imminent_time_)
                        dispatch(FsmEvent::TRAJ_UNSAFE_IMMINENT);
                    else
                    {
                        dispatch(FsmEvent::TRAJ_UNSAFE);
                        if (state_ == FsmState::SLOWDOWN &&
                            (this->now() - unsafe_since_).seconds() > brake_timeout_)
                            dispatch(FsmEvent::UNSAFE_TIMEOUT);
                    }
                    if (state_ == FsmState::BRAKE)
                        snap.reset();  // dispatch 已丢弃轨迹
                }
                else if (chk.warn())
                {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                         "Traj marginal in %.2fs (min esdf %.2fm), slowing",
                                         chk.first_warn_time - elapsed, chk.min_distance);
                    // warn 级不超时升级：窄道慢速通过优于趴窝
                    dispatch(FsmEvent::TRAJ_UNSAFE);
                    unsafe_since_ = this->now();  // hard 计时器随 warn 刷新
                }
                else if (state_ == FsmState::SLOWDOWN &&
                         chk.min_distance >= safety_margin_warn_ + safety_exit_hyst_)
                    dispatch(FsmEvent::TRAJ_SAFE);  // 带迟滞退出，防边界抖动
            }
        }

        // 按状态输出控制指令
        geometry_msgs::msg::Twist cmd;
        switch (state_)
        {
        case FsmState::BRAKE:
            break;  // 零指令刹停（阶梯的底）

        case FsmState::EXEC:
        case FsmState::SLOWDOWN:
            if (snap)
            {
                double elapsed = (this->now() - snap->start_time).seconds();
                cmd = tracker_.compute(current_pos_, current_vel_, current_yaw_,
                                       snap->traj, elapsed, true);
                if (state_ == FsmState::SLOWDOWN)
                {
                    cmd.linear.x *= slowdown_factor_;
                    cmd.linear.y *= slowdown_factor_;
                }
                break;
            }
            [[fallthrough]];  // 快照已失效：退回无轨迹行为

        case FsmState::IDLE:
            cmd = tracker_.compute(current_pos_, current_vel_, current_yaw_,
                                   empty_traj_, 0.0, false);
            break;
        }
        cmd_vel_pub_->publish(cmd);
    }

    void publishTrajectory(const sentry_planner::MincoTrajectory &traj, double duration)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "odom";

        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::Marker del;
        del.header = path_msg.header;
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(del);

        visualization_msgs::msg::Marker line;
        line.header = path_msg.header;
        line.ns = "trajectory"; line.id = 0;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.08;
        line.color.g = 1.0; line.color.b = 1.0; line.color.a = 1.0;

        for (double t = 0; t <= duration; t += 0.05)
        {
            Eigen::Vector2d pt = traj.getPosition(t);
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = pt(0); pose.pose.position.y = pt(1);
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);

            geometry_msgs::msg::Point p;
            p.x = pt(0); p.y = pt(1);
            line.points.push_back(p);
        }

        markers.markers.push_back(line);
        path_pub_->publish(path_msg);
        marker_pub_->publish(markers);
    }

    // Components
    std::shared_ptr<SDFMap> sdf_map_;
    std::shared_ptr<EDTEnvironment> edt_env_;
    std::shared_ptr<KinodynamicAstar> kino_astar_;
    sentry_planner::MincoTrajectory minco_traj_;   // 规划工作对象（仅重规划路径使用）
    sentry_planner::MincoTrajectory empty_traj_;   // IDLE 时占位
    sentry_planner::TrajectoryTracker tracker_;

    // State
    Eigen::Vector2d current_pos_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d current_vel_ = Eigen::Vector2d::Zero();
    double current_yaw_ = 0.0;
    bool has_odom_ = false;
    Eigen::Vector2d last_odom_pos_ = Eigen::Vector2d::Zero();
    double last_odom_time_ = -1.0;
    bool twist_seen_ = false;  // 收到过非零 twist 后不再退回位姿差分

    // 按需重规划状态
    double replan_deviation_ = 0.5;
    double replan_refresh_period_ = 1.0;
    double replan_plan_lead_ = 0.15;
    rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};
    Eigen::Vector2d last_planned_goal_ = Eigen::Vector2d::Constant(1e9);

    FsmState state_ = FsmState::IDLE;
    rclcpp::Time unsafe_since_;

    std::mutex traj_mutex_;
    TrajSnapshotPtr active_traj_;

    // 安全监控参数
    double safety_check_horizon_ = 2.0;
    double safety_check_dt_ = 0.03;
    double safety_margin_warn_ = 0.20;
    double safety_margin_hard_ = 0.15;
    double safety_exit_hyst_ = 0.05;
    double safety_self_ignore_r_ = 0.35;
    double safety_imminent_time_ = 0.8;
    double slowdown_factor_ = 0.4;
    double brake_timeout_ = 0.5;

    Eigen::Vector2d final_goal_ = Eigen::Vector2d::Zero();
    bool has_final_goal_ = false;

    std::vector<Eigen::Vector2d> global_waypoints_;
    int current_waypoint_idx_ = 0;
    bool has_global_path_ = false;

    // ROS
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr ctrl_timer_, replan_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SentryLocalPlannerNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
