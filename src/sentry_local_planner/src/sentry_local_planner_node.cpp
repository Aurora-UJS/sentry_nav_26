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
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <optional>
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
        this->declare_parameter<double>("minco_opt.lambda_oneway", 8.0);
        this->declare_parameter<double>("minco_opt.dist0_vel_k",   0.0);

        double mv = 3.0, ma = 3.0;
        this->get_parameter("search.max_vel", mv);
        this->get_parameter("search.max_acc", ma);
        minco_traj_.setOptimizer(
            edt_env_.get(),
            this->get_parameter("minco_opt.lambda_smooth").as_double(),
            this->get_parameter("minco_opt.lambda_col").as_double(),
            this->get_parameter("minco_opt.lambda_feas").as_double(),
            this->get_parameter("minco_opt.dist0").as_double(),
            this->get_parameter("minco_opt.dist0_vel_k").as_double(),
            mv, ma,
            this->get_parameter("minco_opt.robot_radius").as_double(),
            this->get_parameter("minco_opt.num_samples").as_int(),
            this->get_parameter("minco_opt.max_iter").as_int(),
            this->get_parameter("minco_opt.max_time_ms").as_double() / 1000.0,
            this->get_parameter("minco_opt.lambda_oneway").as_double());

        // --- 静态可通行性标注层 (free/obstacle/oneway), OPT-IN: 空路径/加载失败 → 不挂载 ---
        this->declare_parameter<std::string>("traversability.yaml_path", "");
        std::string trav_yaml = this->get_parameter("traversability.yaml_path").as_string();
        if (!trav_yaml.empty())
        {
            if (edt_env_->loadTraversability(trav_yaml))
                RCLCPP_INFO(this->get_logger(), "Traversability layer loaded: %s", trav_yaml.c_str());
            else
                RCLCPP_WARN(this->get_logger(),
                            "Traversability yaml_path set but load failed (layer disabled): %s",
                            trav_yaml.c_str());
        }

        // --- Init TrajectoryTracker ---
        this->declare_parameter<double>("controller.frequency", 50.0);
        this->declare_parameter<double>("controller.spin_rate", 3.0);
        this->declare_parameter<double>("controller.corridor_enter_dist", 0.55);
        this->declare_parameter<double>("controller.corridor_exit_dist", 0.80);
        this->declare_parameter<double>("controller.corridor_preview_time", 1.2);
        this->declare_parameter<double>("controller.corridor_side_probe", 0.30);
        this->declare_parameter<double>("controller.wz_slew_rate", 8.0);
        this->declare_parameter<double>("controller.corridor_speed", 0.8);
        this->declare_parameter<double>("controller.corridor_align_speed", 0.2);
        this->declare_parameter<double>("controller.corridor_align_tol_deg", 20.0);
        this->declare_parameter<double>("controller.corridor_lookahead", 0.6);
        this->declare_parameter<double>("controller.slope_enter_deg", 7.0);
        this->declare_parameter<double>("controller.slope_exit_deg", 4.0);
        this->declare_parameter<double>("controller.yaw_align_kp", 3.0);
        this->declare_parameter<double>("controller.max_align_wz", 2.5);
        this->declare_parameter<double>("controller.min_turn_radius", 0.6);
        this->declare_parameter<int>("mpc.horizon", 10);
        this->declare_parameter<double>("mpc.q_pos", 10.0);
        this->declare_parameter<double>("mpc.q_vel", 1.0);
        this->declare_parameter<double>("mpc.r_acc", 0.1);

        double ctrl_freq = this->get_parameter("controller.frequency").as_double();
        // world: 输出 odom 系速度到 /cmd_vel_world，由 chassis_cmd_node（真车=电控）
        // 用高频陀螺 yaw 旋转；body: 旧行为，本节点用 LIO yaw 旋转后直发 /cmd_vel
        this->declare_parameter<std::string>("controller.cmd_frame", "world");
        auto tcfg = loadTrackerConfig();
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
        this->declare_parameter<double>("safety.brake_settle_time", 1.0);
        this->declare_parameter<double>("safety.brake_settle_vel", 0.1);
        safety_check_horizon_ = this->get_parameter("safety.check_horizon").as_double();
        safety_check_dt_      = this->get_parameter("safety.check_dt").as_double();
        safety_margin_warn_   = this->get_parameter("safety.margin_warn").as_double();
        safety_margin_hard_   = this->get_parameter("safety.margin_hard").as_double();
        safety_exit_hyst_     = this->get_parameter("safety.exit_hysteresis").as_double();
        safety_self_ignore_r_ = this->get_parameter("safety.self_ignore_radius").as_double();
        safety_imminent_time_ = this->get_parameter("safety.imminent_time").as_double();
        slowdown_factor_      = this->get_parameter("safety.slowdown_factor").as_double();
        brake_timeout_        = this->get_parameter("safety.brake_timeout").as_double();
        brake_settle_time_    = this->get_parameter("safety.brake_settle_time").as_double();
        brake_settle_vel_     = this->get_parameter("safety.brake_settle_vel").as_double();

        // --- 按需重规划参数 ---
        this->declare_parameter<double>("replan.deviation", 0.5);
        this->declare_parameter<double>("replan.refresh_period", 1.0);
        this->declare_parameter<double>("replan.plan_lead", 0.15);
        this->declare_parameter<double>("replan.jps_lookahead", 2.5);
        this->declare_parameter<double>("replan.goal_clearance", 0.45);
        this->declare_parameter<double>("replan.pursuit_speed", 1.5);
        this->declare_parameter<int>("replan.crawl_fail_threshold", 5);
        replan_deviation_      = this->get_parameter("replan.deviation").as_double();
        replan_refresh_period_ = this->get_parameter("replan.refresh_period").as_double();
        replan_plan_lead_      = this->get_parameter("replan.plan_lead").as_double();
        jps_lookahead_         = this->get_parameter("replan.jps_lookahead").as_double();
        goal_clearance_        = this->get_parameter("replan.goal_clearance").as_double();
        pursuit_speed_         = this->get_parameter("replan.pursuit_speed").as_double();
        crawl_fail_threshold_  = this->get_parameter("replan.crawl_fail_threshold").as_int();

        // --- 卡滞检测参数 ---
        this->declare_parameter<double>("stuck.timeout", 0.8);
        this->declare_parameter<double>("stuck.inject_dist", 0.5);
        stuck_timeout_     = this->get_parameter("stuck.timeout").as_double();
        stuck_inject_dist_ = this->get_parameter("stuck.inject_dist").as_double();
        world_frame_cmd_   = tcfg.world_frame_cmd;
        corridor_enter_dist_ = tcfg.corridor_enter_dist;

        // --- Subscribers ---
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                current_pos_(0) = msg->pose.pose.position.x;
                current_pos_(1) = msg->pose.pose.position.y;
                double qw = msg->pose.pose.orientation.w;
                double qz = msg->pose.pose.orientation.z;
                current_yaw_ = 2.0 * atan2(qz, qw);

                // 机身倾角 = 体轴 z 与世界 z 夹角（坡道模式检测用）。
                // R(2,2) = 1 - 2(qx² + qy²)，只含横滚/俯仰，与 yaw 无关
                double qx = msg->pose.pose.orientation.x;
                double qy = msg->pose.pose.orientation.y;
                double r22 = 1.0 - 2.0 * (qx * qx + qy * qy);
                current_tilt_ = acos(std::clamp(r22, -1.0, 1.0));

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
                // 多线程下与重规划线程并发读 global_waypoints_，重建必须持锁：
                // 旧代码 clear+emplace 会让重规划线程迭代到被清空的 vector（崩溃）。
                std::vector<Eigen::Vector2d> wps;
                wps.reserve(msg->poses.size());
                for (auto &pose : msg->poses)
                    wps.emplace_back(pose.pose.position.x, pose.pose.position.y);
                {
                    std::lock_guard<std::mutex> lk(global_path_mutex_);
                    global_waypoints_ = std::move(wps);
                    has_global_path_ = !global_waypoints_.empty();
                }
                RCLCPP_INFO(this->get_logger(), "Received global path: %zu waypoints",
                            msg->poses.size());
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
        // 重规划独立回调组：与 50Hz 控制/odom 回调组（默认组）并发，
        // 规划耗时不再占用控制时隙。默认组内部仍串行（互斥），互不打架。
        replan_cb_group_ =
            this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        replan_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / replan_freq),
            std::bind(&SentryLocalPlannerNode::replanCallback, this),
            replan_cb_group_);

        RCLCPP_INFO(this->get_logger(),
                    "Planner initialized: minco+mpc, ctrl@%.0fHz, safety horizon %.1fs",
                    ctrl_freq, safety_check_horizon_);

        // 参数热重载（仅 debug 模式）：比赛/真车运行时锁死参数，避免误改参数
        // 扰动正在执行的轨迹；只有 debug_mode:=true 才注册 set 回调。
        this->declare_parameter<bool>("debug_mode", false);
        debug_mode_ = this->get_parameter("debug_mode").as_bool();
        if (debug_mode_)
        {
            this->add_on_set_parameters_callback(
                [this](const std::vector<rclcpp::Parameter> &ps) {
                    return this->onParamChange(ps);
                });
            RCLCPP_WARN(this->get_logger(),
                        "debug_mode=true: parameter hot-reload ENABLED (ros2 param set 即时生效)");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),
                        "debug_mode=false: parameter hot-reload disabled (改参需重启)");
        }
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

    /** 从参数服务器组装 TrajectoryTracker 配置 (init 与热重载共用) */
    sentry_planner::TrajectoryTracker::Config loadTrackerConfig()
    {
        sentry_planner::TrajectoryTracker::Config tcfg;
        double mv = 3.0, ma = 3.0;
        this->get_parameter("search.max_vel", mv);
        this->get_parameter("search.max_acc", ma);
        tcfg.max_vel = mv;
        tcfg.max_acc = ma;
        tcfg.ctrl_dt = 1.0 / this->get_parameter("controller.frequency").as_double();
        tcfg.spin_rate = this->get_parameter("controller.spin_rate").as_double();
        tcfg.corridor_enter_dist = this->get_parameter("controller.corridor_enter_dist").as_double();
        tcfg.corridor_exit_dist = this->get_parameter("controller.corridor_exit_dist").as_double();
        tcfg.corridor_preview_time = this->get_parameter("controller.corridor_preview_time").as_double();
        tcfg.corridor_side_probe = this->get_parameter("controller.corridor_side_probe").as_double();
        tcfg.wz_slew_rate = this->get_parameter("controller.wz_slew_rate").as_double();
        tcfg.corridor_speed = this->get_parameter("controller.corridor_speed").as_double();
        tcfg.corridor_align_speed = this->get_parameter("controller.corridor_align_speed").as_double();
        tcfg.corridor_align_tol =
            this->get_parameter("controller.corridor_align_tol_deg").as_double() * M_PI / 180.0;
        tcfg.corridor_lookahead = this->get_parameter("controller.corridor_lookahead").as_double();
        tcfg.slope_enter_rad =
            this->get_parameter("controller.slope_enter_deg").as_double() * M_PI / 180.0;
        tcfg.slope_exit_rad =
            this->get_parameter("controller.slope_exit_deg").as_double() * M_PI / 180.0;
        tcfg.yaw_align_kp = this->get_parameter("controller.yaw_align_kp").as_double();
        tcfg.max_align_wz = this->get_parameter("controller.max_align_wz").as_double();
        tcfg.min_turn_radius = this->get_parameter("controller.min_turn_radius").as_double();
        tcfg.mpc_horizon = this->get_parameter("mpc.horizon").as_int();
        tcfg.mpc_q_pos = this->get_parameter("mpc.q_pos").as_double();
        tcfg.mpc_q_vel = this->get_parameter("mpc.q_vel").as_double();
        tcfg.mpc_r_acc = this->get_parameter("mpc.r_acc").as_double();
        // 注意: cmd_frame 热改会造成输出帧与发布话题不一致，重启才安全。
        tcfg.world_frame_cmd =
            (this->get_parameter("controller.cmd_frame").as_string() == "world");
        return tcfg;
    }

    /** 参数热重载回调（仅 debug_mode=true 时注册）:
     *  controller./mpc./safety./replan./stuck. 前缀参数 ros2 param set 即时生效 */
    rcl_interfaces::msg::SetParametersResult onParamChange(
        const std::vector<rclcpp::Parameter> &params)
    {
        bool tracker_dirty = false, safety_dirty = false, replan_dirty = false,
             stuck_dirty = false;
        for (const auto &p : params)
        {
            const std::string &n = p.get_name();
            if (n.rfind("controller.", 0) == 0 || n.rfind("mpc.", 0) == 0)
                tracker_dirty = true;
            else if (n.rfind("safety.", 0) == 0)
                safety_dirty = true;
            else if (n.rfind("replan.", 0) == 0)
                replan_dirty = true;
            else if (n.rfind("stuck.", 0) == 0)
                stuck_dirty = true;
        }

        if (tracker_dirty)
        {
            // 50Hz 控制线程并发读 cfg_：字段为对齐 double，单字段写入原子，
            // 跨字段撕裂最多影响一拍，调参场景可接受。
            auto tcfg = loadTrackerConfig();
            tracker_.updateConfig(tcfg);
            corridor_enter_dist_ = tcfg.corridor_enter_dist;
            RCLCPP_INFO(this->get_logger(),
                        "Tracker config hot-reloaded: spin=%.1f, enter=%.2f, "
                        "min_turn_radius=%.2f, wz_slew=%.1f",
                        tcfg.spin_rate, tcfg.corridor_enter_dist,
                        tcfg.min_turn_radius, tcfg.wz_slew_rate);
        }
        if (safety_dirty)
        {
            safety_check_horizon_ = this->get_parameter("safety.check_horizon").as_double();
            safety_check_dt_      = this->get_parameter("safety.check_dt").as_double();
            safety_margin_warn_   = this->get_parameter("safety.margin_warn").as_double();
            safety_margin_hard_   = this->get_parameter("safety.margin_hard").as_double();
            safety_exit_hyst_     = this->get_parameter("safety.exit_hysteresis").as_double();
            safety_self_ignore_r_ = this->get_parameter("safety.self_ignore_radius").as_double();
            safety_imminent_time_ = this->get_parameter("safety.imminent_time").as_double();
            slowdown_factor_      = this->get_parameter("safety.slowdown_factor").as_double();
            brake_timeout_        = this->get_parameter("safety.brake_timeout").as_double();
            brake_settle_time_    = this->get_parameter("safety.brake_settle_time").as_double();
            brake_settle_vel_     = this->get_parameter("safety.brake_settle_vel").as_double();
            RCLCPP_INFO(this->get_logger(),
                        "Safety params hot-reloaded: warn=%.2f hard=%.2f",
                        safety_margin_warn_, safety_margin_hard_);
        }
        if (replan_dirty)
        {
            replan_deviation_      = this->get_parameter("replan.deviation").as_double();
            replan_refresh_period_ = this->get_parameter("replan.refresh_period").as_double();
            replan_plan_lead_      = this->get_parameter("replan.plan_lead").as_double();
            jps_lookahead_         = this->get_parameter("replan.jps_lookahead").as_double();
            goal_clearance_        = this->get_parameter("replan.goal_clearance").as_double();
            pursuit_speed_         = this->get_parameter("replan.pursuit_speed").as_double();
            crawl_fail_threshold_  = this->get_parameter("replan.crawl_fail_threshold").as_int();
        }
        if (stuck_dirty)
        {
            stuck_timeout_     = this->get_parameter("stuck.timeout").as_double();
            stuck_inject_dist_ = this->get_parameter("stuck.inject_dist").as_double();
        }

        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        return res;
    }

    enum class PlanResult { SUCCESS, FAILED, SKIPPED };

    /** 状态机事件分发：转移由纯函数决定，进入新状态的副作用集中在这里。
     *  多线程下 controlLoop(默认组) 与 replan(重规划组) 都可能调用 → 用 state_mutex_
     *  串行化读-改-写，避免并发转移互相覆盖（丢一次转移会下一拍自愈，但仍应避免）。 */
    void dispatch(FsmEvent e)
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
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
            // 刹停即放弃当前轨迹；恢复靠 PLAN_SUCCESS 或静置超时降级 IDLE
            setActiveTraj(nullptr);
            brake_since_ = this->now();
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
        consec_plan_fail_ = 0;
        // 不在这里清空 global_waypoints_：全局规划器 JPS 只要 ~1ms，新路径常比本回调
        // （与 50Hz 控制/60000pt 点云同组串行）先到；无条件清空会把刚到手的新路径 wipe 掉
        // → 局部规划器永远 "waiting for global path"。改为在 replanCallback 校验路径终点
        // 是否落在当前目标附近：旧路径作废、新路径到达后自然覆盖。
        if ((final_goal_ - current_pos_).norm() <= 2.0 * jps_lookahead_)
            dispatchPlanResult(planToGoal(final_goal_, Eigen::Vector2d::Zero(),
                                          current_pos_, current_vel_));
        else
        {
            // 远目标不直接规划（先验图之外的坑/封闭区只有 JPS 知道）：
            // 丢弃朝旧目标的轨迹，由 replanCallback 在收到全局路径后接管
            setActiveTraj(nullptr);
            dispatch(FsmEvent::TRAJ_FINISHED);
            RCLCPP_INFO(this->get_logger(), "Far goal (%.1fm), waiting for global path",
                        (final_goal_ - current_pos_).norm());
        }
    }

    /**
     * 路径弧长前瞻点：取 path 上距 p 最近点前方 lookahead 弧长处（段内插值）。
     * 用于 JPS 路径加密跟随 —— 局部规划只做短途避障，不在稀疏 waypoint 间
     * 长距自由规划。点云局部图看不见坑/跌落等负障碍，抄近路会直接掉进
     * 只有先验图才知道的坑里（实测卡死）；JPS 在先验图上避坑，贴着它走
     * 局部层就没有抄近路的空间。
     */
    static Eigen::Vector2d pursuitPointOnPath(const std::vector<Eigen::Vector2d> &path,
                                              const Eigen::Vector2d &p, double lookahead)
    {
        double best_d2 = std::numeric_limits<double>::max();
        int best_i = 0;
        double best_t = 0.0;
        for (int i = 0; i + 1 < (int)path.size(); ++i)
        {
            Eigen::Vector2d ab = path[i + 1] - path[i];
            double L2 = ab.squaredNorm();
            double t = L2 < 1e-12 ? 0.0 : std::clamp((p - path[i]).dot(ab) / L2, 0.0, 1.0);
            double d2 = (path[i] + t * ab - p).squaredNorm();
            if (d2 < best_d2) { best_d2 = d2; best_i = i; best_t = t; }
        }
        Eigen::Vector2d cur = path[best_i] + best_t * (path[best_i + 1] - path[best_i]);
        double remain = lookahead;
        for (int i = best_i; i + 1 < (int)path.size(); ++i)
        {
            Eigen::Vector2d a = (i == best_i) ? cur : path[i];
            Eigen::Vector2d seg = path[i + 1] - a;
            double len = seg.norm();
            if (len >= remain)
                return a + seg / len * remain;
            remain -= len;
        }
        return path.back();
    }

    /**
     * 全局路径在点 p 处的单位切向（前向）。p 通常是 pursuitPointOnPath 的返回值，
     * 用于把局部轨迹终点速度从 0 改成"沿路径前进"，消除每个 pursuit 点的人工刹停。
     */
    static Eigen::Vector2d pathTangent(const std::vector<Eigen::Vector2d> &path,
                                       const Eigen::Vector2d &p)
    {
        if (path.size() < 2) return Eigen::Vector2d::Zero();
        double best_d2 = std::numeric_limits<double>::max();
        int best_i = 0;
        for (int i = 0; i + 1 < (int)path.size(); ++i)
        {
            Eigen::Vector2d ab = path[i + 1] - path[i];
            double L2 = ab.squaredNorm();
            double t = L2 < 1e-12 ? 0.0 : std::clamp((p - path[i]).dot(ab) / L2, 0.0, 1.0);
            double d2 = (path[i] + t * ab - p).squaredNorm();
            if (d2 < best_d2) { best_d2 = d2; best_i = i; }
        }
        Eigen::Vector2d d = path[best_i + 1] - path[best_i];
        return d.norm() < 1e-12 ? Eigen::Vector2d::Zero() : d.normalized();
    }

    /**
     * 窄道直穿兜底指令（仅 IDLE 且有目标时调用）。
     * 触发条件: 在窄区内 (esdf < corridor_enter) 且 JPS 静态验证路径可跟。
     * 安全分层: 静态层保证基准可行性（标注间隙 ≥ 车宽，JPS 发布前已验证
     * min-clearance）；活图沿爬行线做硬保护 —— 低于 margin_hard 视为真堵死
     * （新障碍/对方机器人），停住等待，交给 occ_timeout 自愈或上层换路。
     */
    std::optional<geometry_msgs::msg::Twist> narrowCrawlCmd()
    {
        if (!has_final_goal_ || !has_global_path_ || global_waypoints_.size() < 2)
        {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                  "Crawl skip: goal=%d path=%zu", has_final_goal_,
                                  global_waypoints_.size());
            return std::nullopt;
        }
        // 触发: 在窄区内 (esdf < corridor_enter)，或规划连败但全局路径仍在
        // （目标被围/幽灵格封路时机器人可能停在开阔处）
        bool in_narrow = edt_env_->isInMap(current_pos_) &&
                         edt_env_->getDistance(current_pos_) < corridor_enter_dist_;
        if (!in_narrow && consec_plan_fail_ < crawl_fail_threshold_)
        {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                  "Crawl skip: esdf %.2f open, fails %d/%d",
                                  edt_env_->getDistance(current_pos_),
                                  consec_plan_fail_, crawl_fail_threshold_);
            return std::nullopt;
        }

        Eigen::Vector2d target =
            pursuitPointOnPath(global_waypoints_, current_pos_, 0.6);
        Eigen::Vector2d d = target - current_pos_;
        double len = d.norm();
        if (len < 0.05)
            return std::nullopt;
        d /= len;
        // 爬行线活图硬保护（近端豁免机器人自身已占据区域）
        for (double s = 0.15; s <= len; s += 0.05)
        {
            Eigen::Vector2d p = current_pos_ + d * s;
            if (!edt_env_->isInMap(p) || edt_env_->getDistance(p) < safety_margin_hard_)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "Narrow crawl blocked at (%.2f, %.2f) esdf<%.2f - waiting",
                                     p(0), p(1), safety_margin_hard_);
                return std::nullopt;
            }
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Narrow crawl along global path toward (%.2f, %.2f)",
                             target(0), target(1));
        return tracker_.computeCrawl(current_pos_, current_yaw_, target);
    }

    void dispatchPlanResult(PlanResult res)
    {
        if (res == PlanResult::SUCCESS)
        {
            consec_plan_fail_ = 0;
            dispatch(FsmEvent::PLAN_SUCCESS);
        }
        else if (res == PlanResult::FAILED)
        {
            ++consec_plan_fail_;  // SKIPPED 不计: 那是"没尝试"不是"失败"
            dispatch(FsmEvent::PLAN_FAIL);
            // IDLE 也要出声: 曾因此静默失败 160s 无从诊断（夹口 A* 无路趴窝）
            if (state_ == FsmState::IDLE)
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "Replan failed, state=IDLE (robot parked)");
            if (state_ != FsmState::IDLE)
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Replan failed, state=%s",
                                     sentry_planner::fsmStateName(state_));
        }
    }

    PlanResult planToGoal(Eigen::Vector2d goal_pt, Eigen::Vector2d goal_vel,
                          Eigen::Vector2d start_pos, Eigen::Vector2d start_vel)
    {
        // 规划工作对象 kino_astar_/minco_traj_ 非线程安全，而 goalCallback(默认组) 与
        // replanCallback(重规划组) 都可能触发规划 → plan_mutex_ 串行化。
        std::lock_guard<std::mutex> lk(plan_mutex_);
        Eigen::Vector2d diff = goal_pt - start_pos;
        // 执行中让最后一段轨迹自然跑完（near_end 邻域内不打断）；
        // 无轨迹（IDLE/BRAKE 后）必须照常规划——旧版无条件 SKIPPED 会让
        // 近距目标（如隔台沿 0.87m 的 pursuit 点）永远不被尝试
        if (diff.norm() < 1.0 && getActiveTraj()) return PlanResult::SKIPPED;

        double local_range = 7.5;
        this->get_parameter("sdf_map.local_update_range_x", local_range);
        local_range -= 0.5;
        if (diff.norm() > local_range)
            goal_pt = start_pos + diff.normalized() * local_range;

        Eigen::Vector2d map_origin, map_size;
        sdf_map_->getRegion(map_origin, map_size);
        for (int i = 0; i < 2; ++i)
            goal_pt(i) = std::clamp(goal_pt(i), map_origin(i) + 0.5, map_origin(i) + map_size(i) - 0.5);

        // 局部目标推离障碍：pursuit 点沿贴墙的全局路径取时可能钉在墙皮/膨胀区里
        // （先验图间隙够、局部图更紧），A* 对不可达目标反复失败 → 原地蠕动。
        // 沿局部 ESDF 数值梯度推到 goal_clearance 再交给 A*。
        {
            const double res = sdf_map_->getResolution();
            for (int iter = 0; iter < 20; ++iter)
            {
                if (sdf_map_->getDistance(goal_pt) >= goal_clearance_)
                    break;
                Eigen::Vector2d grad(
                    sdf_map_->getDistance(goal_pt + Eigen::Vector2d(res, 0)) -
                        sdf_map_->getDistance(goal_pt - Eigen::Vector2d(res, 0)),
                    sdf_map_->getDistance(goal_pt + Eigen::Vector2d(0, res)) -
                        sdf_map_->getDistance(goal_pt - Eigen::Vector2d(0, res)));
                if (grad.norm() < 1e-6)
                    break;
                goal_pt += grad.normalized() * res;
            }
        }

        // 起点状态：执行中且跟踪良好时，从当前轨迹的参考点 (elapsed + lead) 续接，
        // 而不是从实测状态重启 —— 否则每次重规划都把参考速度打回起步段，
        // 10Hz 重规划下参考速度永远爬不上去（实测 5.4m 直线平均有效速度仅 0.27m/s）。
        // 跟踪偏差过大时回退到实测状态重锚（MPC 已跟不上参考，续接无意义）。
        Eigen::Vector2d start_acc = Eigen::Vector2d::Zero();
        rclcpp::Time traj_t0 = this->now();
        auto prev = getActiveTraj();
        if (prev && (state_ == FsmState::EXEC || state_ == FsmState::SLOWDOWN))
        {
            double elapsed = (this->now() - prev->start_time).seconds();
            double t_ref = std::min(elapsed + replan_plan_lead_, prev->duration);
            Eigen::Vector2d p_ref = prev->traj.getPosition(t_ref);
            if ((p_ref - start_pos).norm() < replan_deviation_)
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
                                          goal_pt, goal_vel, true);
        if (result == KinodynamicAstar::NO_PATH)
        {
            // 阶段归因: 失败必须可诊断（曾静默失败 762 次无从归因）
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Plan stage A*: NO_PATH, start(%.2f,%.2f) -> goal(%.2f,%.2f) esdf %.2f",
                                 start_pos(0), start_pos(1), goal_pt(0), goal_pt(1),
                                 sdf_map_->getDistance(goal_pt));
            return PlanResult::FAILED;
        }

        double ts = 0.1;  // A* 采样间隔(s): getSamples 用其估算段数。曾未初始化 → UB (被 max(8,·) 掩盖)
        std::vector<Eigen::Vector2d> point_set, start_end_derivatives;
        kino_astar_->getSamples(ts, point_set, start_end_derivatives);
        if (point_set.size() < 2)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Plan stage sampling: %zu points (A* status %d)",
                                 point_set.size(), result);
            return PlanResult::FAILED;
        }

        // MINCO
        double max_vel = 3.0, max_acc = 3.0;
        this->get_parameter("search.max_vel", max_vel);
        this->get_parameter("search.max_acc", max_acc);

        Eigen::Vector2d sv = start_end_derivatives.size() >= 2 ? start_end_derivatives[0] : start_vel;
        Eigen::Vector2d ev = start_end_derivatives.size() >= 2 ? start_end_derivatives[1] : Eigen::Vector2d::Zero();
        Eigen::Vector2d sa = start_end_derivatives.size() >= 4 ? start_end_derivatives[2] : start_acc;
        Eigen::Vector2d ea = start_end_derivatives.size() >= 4 ? start_end_derivatives[3] : Eigen::Vector2d::Zero();

        minco_traj_.setup(point_set, sv, sa, ev, ea, max_vel, max_acc, this->now().seconds());
        if (minco_traj_.empty())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Plan stage MINCO: empty trajectory from %zu points (A* status %d)",
                                 point_set.size(), result);
            return PlanResult::FAILED;
        }

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

        // 多线程快照：本回调与 odom/goal/global_path 回调组并发，共享状态在入口
        // 一次性拷贝，避免中途被改写（尤其 global_waypoints_ 的迭代会撞上 clear）。
        Eigen::Vector2d pos_snap = current_pos_;
        Eigen::Vector2d vel_snap = current_vel_;
        Eigen::Vector2d goal_snap = final_goal_;
        std::vector<Eigen::Vector2d> wp_snap;
        bool has_global_snap = false;
        {
            std::lock_guard<std::mutex> lk(global_path_mutex_);
            wp_snap = global_waypoints_;
            has_global_snap = has_global_path_;
        }

        if ((goal_snap - pos_snap).norm() < 1.0) return;

        Eigen::Vector2d local_goal = goal_snap;
        Eigen::Vector2d local_goal_vel = Eigen::Vector2d::Zero();
        // 路径有效性：终点必须落在当前目标附近（JPS 路径终点=目标，push 后处理不动端点）。
        // 目标变了但新路径还没到 → 旧路径作废，视同无路径等待；新路径到达后自然覆盖。
        bool path_valid = has_global_snap && wp_snap.size() >= 2 &&
                          (wp_snap.back() - goal_snap).norm() < 1.0;
        if (path_valid)
        {
            local_goal = pursuitPointOnPath(wp_snap, pos_snap, jps_lookahead_);
            // pursuit 点是中途点而非终点：终点速度取全局路径切向 × pursuit_speed，
            // 不再在每个 2.5m 前瞻点强制刹停。终点速度>0 → 局部轨迹保持巡航、
            // 减速段缩短 → 峰值加速度更缓（配合 max_acc 安全阀，不易侧翻）。
            local_goal_vel = pathTangent(wp_snap, local_goal) * pursuit_speed_;
        }
        else if ((goal_snap - pos_snap).norm() > 2.0 * jps_lookahead_)
        {
            // 远目标必须有全局路径兜底：先验图知道坑/跌落/封闭区，局部点云图
            // 不知道。JPS 未出路径（未达/不可达）时不做长距自由规划，原地等待。
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Far goal (%.1fm) without global path - waiting for JPS "
                                 "(unreachable in prior map?)",
                                 (goal_snap - pos_snap).norm());
            return;
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
                (snap->traj.getPosition(std::min(elapsed, snap->duration)) - pos_snap).norm();
            need_replan = deviation > replan_deviation_ ||
                          (local_goal - last_planned_goal_).norm() > 0.3 ||
                          (this->now() - last_plan_time_).seconds() > replan_refresh_period_;
        }

        if (need_replan)
            dispatchPlanResult(planToGoal(local_goal, local_goal_vel, pos_snap, vel_snap));
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

                // 违例持续性计数：坡脚等贴墙处 ESDF 会单拍闪烁（自转幽灵障碍，
                // occ_timeout 3.5s 自愈），旧逻辑对单拍违例立即转移状态，造成
                // 50ms 周期 BRAKE↔EXEC↔SLOWDOWN 抖动、车在坡脚表现为卡死。
                // 真实障碍每拍持续存在 → 要求连续 2 拍(40ms) 才转移；
                // 1.5m/s 下多走 6cm，可忽略；单拍幽灵不再引发急刹。
                const bool warn_now = chk.warn() || chk.hard();
                warn_streak_ = warn_now ? warn_streak_ + 1 : 0;
                const bool hard_imminent =
                    chk.hard() && (chk.first_hard_time - elapsed) < safety_imminent_time_;
                hard_streak_ = hard_imminent ? hard_streak_ + 1 : 0;

                if (hard_streak_ >= 2)
                {
                    // 连续迫近硬违例 → 刹停（真实碰撞路径，单拍闪烁不会到这）
                    Eigen::Vector2d hp = snap->traj.getPosition(chk.first_hard_time);
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                         "Traj hits obstacle in %.2fs (min esdf %.2fm) at (%.2f, %.2f), robot (%.2f, %.2f)",
                                         chk.first_hard_time - elapsed, chk.min_distance,
                                         hp(0), hp(1), current_pos_(0), current_pos_(1));
                    dispatch(FsmEvent::TRAJ_UNSAFE_IMMINENT);
                    if (state_ == FsmState::BRAKE)
                        snap.reset();  // dispatch 已丢弃轨迹
                }
                else if (chk.hard())
                {
                    // 非迫近硬违例（或迫近首拍）：降速；SLOWDOWN 持续超时升级
                    Eigen::Vector2d hp = snap->traj.getPosition(chk.first_hard_time);
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                         "Traj hits obstacle in %.2fs (min esdf %.2fm) at (%.2f, %.2f), robot (%.2f, %.2f)",
                                         chk.first_hard_time - elapsed, chk.min_distance,
                                         hp(0), hp(1), current_pos_(0), current_pos_(1));
                    dispatch(FsmEvent::TRAJ_UNSAFE);
                    if (state_ == FsmState::SLOWDOWN &&
                        (this->now() - unsafe_since_).seconds() > brake_timeout_)
                        dispatch(FsmEvent::UNSAFE_TIMEOUT);
                }
                else if (warn_streak_ >= 2)
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
            // 刹停完成后降级 IDLE: BRAKE 不能是吸收态——规划连败时唯一的
            // 恢复路径是回到 IDLE 让兜底行为（窄道爬行）接管（台沿死锁实证）
            if (has_final_goal_ &&
                (this->now() - brake_since_).seconds() > brake_settle_time_ &&
                current_vel_.norm() < brake_settle_vel_)
                dispatch(FsmEvent::BRAKE_SETTLED);
            break;  // 零指令刹停（阶梯的底）

        case FsmState::EXEC:
        case FsmState::SLOWDOWN:
            if (snap)
            {
                double elapsed = (this->now() - snap->start_time).seconds();
                cmd = tracker_.compute(current_pos_, current_vel_, current_yaw_, current_tilt_,
                                       snap->traj, elapsed, true);
                // 对齐模式下不再乘 slowdown_factor：warn 阈值按自转扫掠圆
                // (半对角线) 标定，对齐直行的有效半宽更小，且模式本身已是
                // 恒低速；再乘 0.4 就是把窄道压成蠕动。hard/BRAKE 不受影响。
                if (state_ == FsmState::SLOWDOWN && !tracker_.inAlignMode())
                {
                    cmd.linear.x *= slowdown_factor_;
                    cmd.linear.y *= slowdown_factor_;
                }
                break;
            }
            [[fallthrough]];  // 快照已失效：退回无轨迹行为

        case FsmState::IDLE:
            // 僵尸目标清理: BRAKE 下轨迹恒空，GOAL_REACHED 在轨迹结束路径里
            // 实际不可达；BRAKE_SETTLED 降级到这里后目标已在脚下时直接收尾
            if (has_final_goal_ && !snap && (final_goal_ - current_pos_).norm() < 1.0)
            {
                RCLCPP_INFO(this->get_logger(), "Goal reached (idle)");
                has_final_goal_ = false;
                break;
            }
            cmd = tracker_.compute(current_pos_, current_vel_, current_yaw_, current_tilt_,
                                   empty_traj_, 0.0, false);
            // 窄道直穿兜底: 活图有效间隙 ∈ (margin_hard, accept_clearance) 的
            // 通道 A* 判死、爬行硬保护判活 —— 规划管线出不来轨迹时沿静态验证
            // 过间隙的 JPS 路径爬行硬穿，防原地趴窝（实测 545s 静止）。
            if (auto crawl = narrowCrawlCmd())
                cmd = *crawl;
            break;
        }
        cmd_vel_pub_->publish(cmd);

        // 对齐模式切换日志（量化验证用：grep "Align mode"）
        if (tracker_.inAlignMode() != align_mode_prev_)
        {
            align_mode_prev_ = tracker_.inAlignMode();
            RCLCPP_INFO(this->get_logger(), "Align mode %s (narrow=%d slope=%d tilt=%.1fdeg)",
                        align_mode_prev_ ? "ON" : "off",
                        tracker_.inNarrowMode(), tracker_.inSlopeMode(),
                        current_tilt_ * 180.0 / M_PI);
        }

        // 卡滞检测（本体感知）：指令在动而车没动 → 被雷达扫不到的小坎/异物顶住。
        // 感知层对低矮障碍与负障碍天然盲，唯一可靠信号就是"推不动"本身。
        // 处置：在受阻方向注入虚拟障碍块 → A* 自然侧移绕行；occ_timeout 自动过期。
        // 窄道对齐模式内禁用：缝里推不动时往前方注障碍会堵死唯一通路
        // （实测 8 次注入聚在夹口 → A* 无路可绕 → BRAKE 风暴），蹭墙慢挤优于自堵。
        if ((state_ == FsmState::EXEC || state_ == FsmState::SLOWDOWN) &&
            !tracker_.inAlignMode())
        {
            double cmd_norm = std::hypot(cmd.linear.x, cmd.linear.y);
            if (cmd_norm > 0.15 && current_vel_.norm() < 0.08)
            {
                if (stuck_since_.nanoseconds() == 0)
                    stuck_since_ = this->now();
                else if ((this->now() - stuck_since_).seconds() > stuck_timeout_ &&
                         (this->now() - last_stuck_inject_).seconds() > 1.0)
                {
                    // cmd_frame=world 时 cmd.linear 即 odom 系；body 模式旋转回 odom 系
                    Eigen::Vector2d dir(cmd.linear.x, cmd.linear.y);
                    if (!world_frame_cmd_)
                    {
                        double c = std::cos(current_yaw_), s = std::sin(current_yaw_);
                        dir = Eigen::Vector2d(c * dir(0) - s * dir(1), s * dir(0) + c * dir(1));
                    }
                    Eigen::Vector2d ob = current_pos_ + dir.normalized() * stuck_inject_dist_;
                    sdf_map_->getCore().addVirtualObstacle(ob, 0.15, this->now().seconds());
                    RCLCPP_WARN(this->get_logger(),
                                "Stuck: cmd %.2f m/s but vel %.2f m/s for %.1fs - "
                                "inject virtual obstacle at (%.2f, %.2f) to sidestep",
                                cmd_norm, current_vel_.norm(), stuck_timeout_, ob(0), ob(1));
                    last_stuck_inject_ = this->now();
                    stuck_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
                }
            }
            else
                stuck_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        }
        else
            stuck_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
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
    double current_tilt_ = 0.0;
    bool align_mode_prev_ = false;
    double corridor_enter_dist_ = 0.55;
    double brake_settle_time_ = 1.0;
    double brake_settle_vel_ = 0.1;
    rclcpp::Time brake_since_{0, 0, RCL_ROS_TIME};
    int consec_plan_fail_ = 0;
    int crawl_fail_threshold_ = 5;
    bool has_odom_ = false;
    Eigen::Vector2d last_odom_pos_ = Eigen::Vector2d::Zero();
    double last_odom_time_ = -1.0;
    bool twist_seen_ = false;  // 收到过非零 twist 后不再退回位姿差分

    // 按需重规划状态
    double replan_deviation_ = 0.5;
    double replan_refresh_period_ = 1.0;
    double replan_plan_lead_ = 0.15;
    double jps_lookahead_ = 2.5;
    double goal_clearance_ = 0.45;
    double pursuit_speed_ = 1.5;   // pursuit 点终点速度幅值 (m/s)

    // 卡滞检测（本体感知脱困）
    double stuck_timeout_ = 0.8;
    double stuck_inject_dist_ = 0.5;
    bool world_frame_cmd_ = false;
    rclcpp::Time stuck_since_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_stuck_inject_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};
    Eigen::Vector2d last_planned_goal_ = Eigen::Vector2d::Constant(1e9);

    FsmState state_ = FsmState::IDLE;
    rclcpp::Time unsafe_since_;

    std::mutex traj_mutex_;
    std::mutex state_mutex_;        // 串行化 dispatch() 状态转移（控制/重规划线程并发调用）
    std::mutex global_path_mutex_;  // 保护 global_waypoints_/has_global_path_（订阅写 vs 重规划读）
    std::mutex plan_mutex_;         // 串行化 planToGoal（保护 kino_astar_/minco_traj_ 工作对象）
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
    int warn_streak_ = 0;   // 连续 warn 拍数（防单拍闪烁抖动）
    int hard_streak_ = 0;   // 连续迫近 hard 拍数（防单拍幽灵急刹）
    bool debug_mode_ = false;  // debug 模式: 参数热重载开关（仅启动时读取）

    Eigen::Vector2d final_goal_ = Eigen::Vector2d::Zero();
    bool has_final_goal_ = false;

    std::vector<Eigen::Vector2d> global_waypoints_;
    bool has_global_path_ = false;

    // ROS
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr ctrl_timer_, replan_timer_;
    rclcpp::CallbackGroup::SharedPtr replan_cb_group_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SentryLocalPlannerNode>();
    node->initialize();
    // 多线程 executor：重规划回调组与 50Hz 控制/odom 回调组并发执行，
    // A*+MINCO (8~40ms) 不再阻塞控制回路（REVIEW #1 单线程丢拍根因）。
    // 共享状态保护见 initialize 内的 callback group 划分 + dispatch/global_path 互斥锁。
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
