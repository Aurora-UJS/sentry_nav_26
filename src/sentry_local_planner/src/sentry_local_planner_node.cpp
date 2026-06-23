/**
 * sentry_local_planner_node: Kinodynamic A* + MINCO + MPC
 */

#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cmath>

#include <plan_env/sdf_map.hpp>
#include <plan_env/edt_environment.hpp>
#include <path_searching/kinodynamic_astar.hpp>

#include <sentry_local_planner/minco_trajectory.hpp>
#include <sentry_local_planner/trajectory_tracker.hpp>

using namespace fast_planner;

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

        // Only the 50 Hz control loop gets its own callback group, so it is never
        // blocked by the heavy replan (A*+MINCO) or the map/ESDF update. Replan, goal,
        // global_path AND every SDFMap callback (esdf/cloud/odom/vis, default group)
        // deliberately share the node's DEFAULT mutually-exclusive group: that
        // serialises the map writes (updateESDF2d / processCloud) against the map reads
        // in A*/MINCO, so planning never observes a half-updated ESDF. Do NOT give the
        // SDFMap callbacks their own group without first adding a map-side lock.
        control_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // --- Init plan_env ---
        sdf_map_ = std::make_shared<SDFMap>();
        sdf_map_->initMap(node_ptr);
        edt_env_ = std::make_shared<EDTEnvironment>();
        edt_env_->setMap(sdf_map_);

        // --- 静态可通行性标注层 (free/obstacle/oneway) ---
        // opt-in: 路径通常由 launch 注入。空路径 → 不挂载 → 行为与未启用时完全一致。
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

        // --- Init A* ---
        kino_astar_ = std::make_shared<KinodynamicAstar>();
        kino_astar_->setParam(node_ptr);
        kino_astar_->setEnvironment(edt_env_.get());
        kino_astar_->init();

        // --- Init MINCO ---
        this->declare_parameter<double>("minco_opt.lambda_smooth", 0.1);
        this->declare_parameter<double>("minco_opt.lambda_col",    8.0);
        this->declare_parameter<double>("minco_opt.lambda_oneway", 8.0);
        this->declare_parameter<double>("minco_opt.lambda_feas",   0.001);
        this->declare_parameter<double>("minco_opt.dist0",         0.05);
        this->declare_parameter<double>("minco_opt.dist0_vel_k",   0.0);
        this->declare_parameter<double>("minco_opt.robot_radius",  0.3);
        this->declare_parameter<int>   ("minco_opt.num_samples",   8);
        this->declare_parameter<int>   ("minco_opt.max_iter",      200);
        this->declare_parameter<double>("minco_opt.max_time_ms",   20.0);

        robot_radius_ = this->get_parameter("minco_opt.robot_radius").as_double();

        // --- Recovery parameters ("走不通就换一条路径": reroute > back-off > rotate) ---
        this->declare_parameter<std::vector<double>>(
            "recovery.fan_offsets_deg", {20.0, 40.0, 60.0, 90.0, 120.0, 150.0, 180.0});
        this->declare_parameter<std::vector<double>>(
            "recovery.fan_radius_scales", {1.0, 0.66, 0.4});
        this->declare_parameter<int>("recovery.max_astar_attempts", 8);
        this->declare_parameter<double>("recovery.candidate_clearance_margin", 0.1);
        this->declare_parameter<double>("recovery.backoff_distance", 0.45);
        this->declare_parameter<double>("recovery.backoff_max_vel_scale", 0.5);
        this->declare_parameter<int>("recovery.backoff_cooldown_cycles", 2);
        this->declare_parameter<bool>("recovery.rotate_enable", true);
        this->declare_parameter<double>("recovery.rotate_yaw_rate", 0.6);
        this->declare_parameter<double>("recovery.rotate_max_time", 1.5);
        this->declare_parameter<int>("recovery.global_cycle_cap", 5);

        fan_offsets_deg_      = this->get_parameter("recovery.fan_offsets_deg").as_double_array();
        fan_radius_scales_    = this->get_parameter("recovery.fan_radius_scales").as_double_array();
        max_astar_attempts_   = this->get_parameter("recovery.max_astar_attempts").as_int();
        cand_clearance_margin_= this->get_parameter("recovery.candidate_clearance_margin").as_double();
        backoff_distance_     = this->get_parameter("recovery.backoff_distance").as_double();
        backoff_max_vel_scale_= this->get_parameter("recovery.backoff_max_vel_scale").as_double();
        backoff_cooldown_cycles_ = this->get_parameter("recovery.backoff_cooldown_cycles").as_int();
        rotate_enable_        = this->get_parameter("recovery.rotate_enable").as_bool();
        rotate_yaw_rate_      = this->get_parameter("recovery.rotate_yaw_rate").as_double();
        rotate_max_time_      = this->get_parameter("recovery.rotate_max_time").as_double();
        global_cycle_cap_     = this->get_parameter("recovery.global_cycle_cap").as_int();

        double mv = 3.0, ma = 3.0;
        this->get_parameter("search.max_vel", mv);
        this->get_parameter("search.max_acc", ma);
        minco_template_.setOptimizer(
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

        // --- Init TrajectoryTracker ---
        this->declare_parameter<double>("controller.frequency", 50.0);
        this->declare_parameter<double>("controller.kp", 2.0);
        this->declare_parameter<double>("controller.lookahead_time", 0.2);
        this->declare_parameter<double>("controller.spin_rate", 0.0);
        this->declare_parameter<double>("controller.narrow_passage_dist", 0.5);
        this->declare_parameter<double>("controller.yaw_align_kp", 1.2);
        this->declare_parameter<double>("controller.max_yaw_rate", 0.8);
        this->declare_parameter<double>("controller.max_yaw_acc", 1.5);
        this->declare_parameter<bool>("controller.follow_path_yaw", true);
        this->declare_parameter<int>("mpc.horizon", 10);
        this->declare_parameter<double>("mpc.q_pos", 10.0);
        this->declare_parameter<double>("mpc.q_vel", 1.0);
        this->declare_parameter<double>("mpc.r_acc", 0.1);

        double ctrl_freq = this->get_parameter("controller.frequency").as_double();

        sentry_planner::TrajectoryTracker::Config tcfg;
        tcfg.max_vel = mv;
        tcfg.max_acc = ma;
        tcfg.ctrl_dt = 1.0 / ctrl_freq;
        tcfg.tracking_kp = this->get_parameter("controller.kp").as_double();
        tcfg.lookahead_time = this->get_parameter("controller.lookahead_time").as_double();
        tcfg.spin_rate = this->get_parameter("controller.spin_rate").as_double();
        tcfg.narrow_passage_dist = this->get_parameter("controller.narrow_passage_dist").as_double();
        tcfg.yaw_align_kp = this->get_parameter("controller.yaw_align_kp").as_double();
        tcfg.max_yaw_rate = this->get_parameter("controller.max_yaw_rate").as_double();
        tcfg.max_yaw_acc = this->get_parameter("controller.max_yaw_acc").as_double();
        tcfg.follow_path_yaw = this->get_parameter("controller.follow_path_yaw").as_bool();
        tcfg.mpc_horizon = this->get_parameter("mpc.horizon").as_int();
        tcfg.mpc_q_pos = this->get_parameter("mpc.q_pos").as_double();
        tcfg.mpc_q_vel = this->get_parameter("mpc.q_vel").as_double();
        tcfg.mpc_r_acc = this->get_parameter("mpc.r_acc").as_double();
        tracker_.init(tcfg, edt_env_.get());

        // --- Subscribers ---
        std::string odom_topic = this->get_parameter("manager.odometry").as_string();
        rclcpp::SubscriptionOptions odom_sub_opts;
        odom_sub_opts.callback_group = control_cb_group_;
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            std::bind(&SentryLocalPlannerNode::odomCallback, this, std::placeholders::_1),
            odom_sub_opts);

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
                resetRecovery();   // a fresh path re-arms NORMAL recovery state
                RCLCPP_INFO(this->get_logger(), "Received global path: %zu waypoints",
                            global_waypoints_.size());
            });

        // --- Publishers ---
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/trajectory", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/planning/trajectory_markers", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // --- Timers ---
        ctrl_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / ctrl_freq),
            std::bind(&SentryLocalPlannerNode::controlLoop, this),
            control_cb_group_);

        this->declare_parameter<double>("replan.frequency", 2.0);
        double replan_freq = this->get_parameter("replan.frequency").as_double();
        replan_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / replan_freq),
            std::bind(&SentryLocalPlannerNode::replanCallback, this));

        RCLCPP_INFO(this->get_logger(), "Planner initialized: minco+mpc, ctrl@%.0fHz", ctrl_freq);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        Eigen::Vector2d pos(msg->pose.pose.position.x, msg->pose.pose.position.y);
        double yaw = yawFromQuaternion(msg->pose.pose.orientation);

        rclcpp::Time stamp(msg->header.stamp);
        if (stamp.nanoseconds() == 0) stamp = this->now();

        Eigen::Vector2d measured_vel = twistToOdomVelocity(msg, yaw);
        if (has_last_odom_)
        {
            double dt = (stamp - last_odom_stamp_).seconds();
            if (dt > 1e-4 && dt < 0.5)
            {
                measured_vel = (pos - last_odom_pos_) / dt;
            }
        }

        if (!isFinite(measured_vel)) measured_vel.setZero();

        // Smooth into the control-group-local accumulator so the new value is a
        // local before we ever touch the shared current_vel_. (odom is the sole
        // writer of current_vel_ and runs mutually-exclusively, so smoothed_vel_
        // mirrors current_vel_ exactly; this keeps current_vel_ access lock-only.)
        if (!has_velocity_estimate_) smoothed_vel_ = measured_vel;
        else smoothed_vel_ = 0.5 * smoothed_vel_ + 0.5 * measured_vel;
        const Eigen::Vector2d new_vel = smoothed_vel_;

        {
            std::lock_guard<std::mutex> lk(state_mutex_);
            current_pos_ = pos;
            current_vel_ = new_vel;
            current_yaw_ = yaw;
            has_odom_ = true;
        }

        has_velocity_estimate_ = true;
        last_odom_pos_ = pos;
        last_odom_stamp_ = stamp;
        has_last_odom_ = true;
    }

    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q)
    {
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    static Eigen::Vector2d twistToOdomVelocity(const nav_msgs::msg::Odometry::SharedPtr &msg, double yaw)
    {
        Eigen::Vector2d vel_body(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
        double cy = std::cos(yaw), sy = std::sin(yaw);
        return Eigen::Vector2d(
            cy * vel_body(0) - sy * vel_body(1),
            sy * vel_body(0) + cy * vel_body(1));
    }

    static bool isFinite(const Eigen::Vector2d &v)
    {
        return std::isfinite(v(0)) && std::isfinite(v(1));
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        Eigen::Vector2d goal(msg->pose.position.x, msg->pose.position.y);
        bool has_odom;
        {
            std::lock_guard<std::mutex> lk(state_mutex_);
            has_odom = has_odom_;
            if (has_odom)
            {
                final_goal_ = goal;
                has_final_goal_ = true;
            }
        }
        if (!has_odom) { RCLCPP_WARN(this->get_logger(), "No odom yet"); return; }
        resetRecovery();   // new goal: stale recovery counts must not pre-escalate
        planToGoal(goal);
    }

    // ---- Recovery state machine ----
    // NORMAL: tracking a plan. REROUTE: primary goal blocked, the angular fan is
    // probing alternatives. BACKOFF: boxed in, retreating along the ESDF gradient.
    // ROTATE: re-aiming in place when no retreat helps. SAFE_IDLE: ladder exhausted,
    // holding position until the world (goal/path/map) changes.
    enum class RecoveryState { NORMAL, REROUTE, BACKOFF, ROTATE, SAFE_IDLE };

    // REROUTE primitive: the exact A*->getSamples->MINCO->swap pipeline, retargeted
    // at an arbitrary (already-clamped) goal point. Returns true ONLY after a
    // successful trajectory swap; on any failure it leaves has_traj_ untouched so
    // the caller (planFailed) owns the terminal fallback / escalation.
    bool tryPlanTo(const Eigen::Vector2d &goal_pt, const Eigen::Vector2d &cur_pos,
                   const Eigen::Vector2d &cur_vel)
    {
        kino_astar_->reset();
        int result = kino_astar_->search(cur_pos, cur_vel, Eigen::Vector2d::Zero(),
                                          goal_pt, Eigen::Vector2d::Zero(), true);
        if (result == KinodynamicAstar::NO_PATH)
            return false;

        double ts;
        std::vector<Eigen::Vector2d> point_set, start_end_derivatives;
        kino_astar_->getSamples(ts, point_set, start_end_derivatives);
        if (point_set.size() < 2)
            return false;

        // MINCO
        double max_vel = 3.0, max_acc = 3.0;
        this->get_parameter("search.max_vel", max_vel);
        this->get_parameter("search.max_acc", max_acc);

        Eigen::Vector2d sv = start_end_derivatives.size() >= 2 ? start_end_derivatives[0] : cur_vel;
        Eigen::Vector2d ev = start_end_derivatives.size() >= 2 ? start_end_derivatives[1] : Eigen::Vector2d::Zero();
        Eigen::Vector2d sa = start_end_derivatives.size() >= 4 ? start_end_derivatives[2] : Eigen::Vector2d::Zero();
        Eigen::Vector2d ea = start_end_derivatives.size() >= 4 ? start_end_derivatives[3] : Eigen::Vector2d::Zero();

        // Build the new trajectory off the hot path. minco_template_ holds the
        // optimizer config (set once at init); copy it so each replan runs setup
        // on a fresh instance and we never publish a half-built trajectory.
        auto new_traj = std::make_shared<sentry_planner::MincoTrajectory>(minco_template_);
        new_traj->setup(point_set, sv, sa, ev, ea, max_vel, max_acc, this->now().seconds());
        if (new_traj->empty())
            return false;

        const double new_duration = new_traj->getDuration();
        const rclcpp::Time new_start = this->now();

        {
            std::lock_guard<std::mutex> lk(traj_mutex_);
            active_traj_     = new_traj;
            traj_duration_   = new_duration;
            traj_start_time_ = new_start;
            has_traj_        = true;
        }
        publishTrajectory(*new_traj, new_duration);
        return true;
    }

    void planToGoal(Eigen::Vector2d goal_pt)
    {
        Eigen::Vector2d cur_pos, cur_vel;
        double cur_yaw = 0.0;
        {
            std::lock_guard<std::mutex> lk(state_mutex_);
            cur_pos = current_pos_;
            cur_vel = current_vel_;
            cur_yaw = current_yaw_;
        }

        Eigen::Vector2d diff = goal_pt - cur_pos;
        // A* near_end tolerance = ceil(1/resolution_astar) * resolution_astar ≈ 1.0m
        // 不要在这个范围内重规划，让最后一段轨迹自然执行完
        if (diff.norm() < 1.0) { onPlanSuccess(); return; }

        double local_range = 7.5;
        this->get_parameter("sdf_map.local_update_range_x", local_range);
        local_range -= 0.5;
        if (diff.norm() > local_range)
            goal_pt = cur_pos + diff.normalized() * local_range;

        Eigen::Vector2d map_origin, map_size;
        sdf_map_->getRegion(map_origin, map_size);
        for (int i = 0; i < 2; ++i)
            goal_pt(i) = std::clamp(goal_pt(i), map_origin(i) + 0.5, map_origin(i) + map_size(i) - 0.5);

        // PRIMARY attempt: drive straight at the (clamped) goal.
        if (tryPlanTo(goal_pt, cur_pos, cur_vel)) { onPlanSuccess(); return; }

        // Blocked: escalate REROUTE -> BACK_OFF -> ROTATE -> SAFE_IDLE.
        planFailed(cur_pos, cur_vel, cur_yaw, goal_pt, local_range, map_origin, map_size);
    }

    // CLEAN EXIT: any successful swap (or near-goal) self-clears recovery so the
    // robot heals automatically the moment a path opens up.
    void onPlanSuccess()
    {
        if (recovery_state_ != RecoveryState::NORMAL)
            RCLCPP_INFO(this->get_logger(), "[recovery] exit -> NORMAL (path found)");
        recovery_state_      = RecoveryState::NORMAL;
        recovery_fail_count_  = 0;
        recovery_cycle_count_ = 0;
        recovery_best_clearance_ = -1.0;
        clearRotate();
    }

    void planFailed(const Eigen::Vector2d &cur_pos, const Eigen::Vector2d &cur_vel,
                    double cur_yaw, const Eigen::Vector2d &primary_goal,
                    double local_range, const Eigen::Vector2d &map_origin,
                    const Eigen::Vector2d &map_size)
    {
        const double base_bearing = std::atan2(primary_goal(1) - cur_pos(1),
                                               primary_goal(0) - cur_pos(0));

        // ---- STAGE 1: REROUTE (PRIMARY — switch to another path) ----
        // Angular fan, smallest |offset| first so the most goal-progressing
        // reroute wins early; bounded by an A* search budget with a cheap EDT
        // pre-filter that rejects most candidates before spending a search.
        int astar_budget = max_astar_attempts_;
        const int signs[2] = {+1, -1};
        bool budget_done = false;
        for (double offset_mag : fan_offsets_deg_)
        {
            if (budget_done) break;
            for (int s = 0; s < 2 && !budget_done; ++s)
            {
                for (double r_scale : fan_radius_scales_)
                {
                    if (astar_budget <= 0) { budget_done = true; break; }
                    double th = base_bearing + signs[s] * offset_mag * M_PI / 180.0;
                    double r  = r_scale * local_range;
                    Eigen::Vector2d c = cur_pos + r * Eigen::Vector2d(std::cos(th), std::sin(th));
                    for (int i = 0; i < 2; ++i)
                        c(i) = std::clamp(c(i), map_origin(i) + 0.5, map_origin(i) + map_size(i) - 0.5);

                    // Cheap pre-filter: reject before paying for an A* search.
                    if (!edt_env_->isInMap(c)) continue;
                    if (edt_env_->getInflateOccupancy(c) != 0) continue;
                    if (edt_env_->getDistance(c) <= robot_radius_ + cand_clearance_margin_) continue;

                    astar_budget--;
                    if (tryPlanTo(c, cur_pos, cur_vel))
                    {
                        RCLCPP_INFO(this->get_logger(), "[recovery] REROUTE succeeded");
                        onPlanSuccess();
                        return;
                    }
                }
            }
        }

        // REROUTE exhausted this tick.
        recovery_fail_count_++;
        if (recovery_state_ != RecoveryState::REROUTE)
            RCLCPP_WARN(this->get_logger(),
                        "[recovery] enter REROUTE: primary goal blocked, fan found no path "
                        "(fail_count=%d)", recovery_fail_count_);
        recovery_state_ = RecoveryState::REROUTE;

        // ---- STAGE 2: BACK_OFF (last resort, only when truly boxed) ----
        // Hysteresis: require >=2 consecutive full-fan failures before retreating;
        // honour the cooldown set after a previous retreat.
        if (recovery_fail_count_ >= 2 && backoff_cooldown_ == 0)
        {
            if (doBackoff(cur_pos, cur_vel, cur_yaw))
            {
                RCLCPP_WARN(this->get_logger(), "[recovery] BACK_OFF: retreating to clearance");
                recovery_state_   = RecoveryState::BACKOFF;
                backoff_cooldown_ = backoff_cooldown_cycles_;
                // Only count toward the give-up cap when the retreat is NOT improving
                // clearance beyond the best this episode. A productive retreat (escaping a
                // deep pocket) resets the cap, so we never strand a robot still making
                // progress; only non-improving thrashing accrues toward SAFE_IDLE.
                if (recovery_last_clearance_ > recovery_best_clearance_ + 0.05)
                {
                    recovery_best_clearance_ = recovery_last_clearance_;
                    recovery_cycle_count_ = 0;
                }
                else
                {
                    recovery_cycle_count_++;
                }
                return;
            }
            // ---- STAGE 3: ROTATE (optional re-aim, only if backoff impossible) ----
            if (rotate_enable_)
            {
                startRotate(base_bearing, cur_yaw);
                RCLCPP_WARN(this->get_logger(), "[recovery] ROTATE: re-aiming in place");
                recovery_state_ = RecoveryState::ROTATE;
                return;
            }
        }

        // ---- GLOBAL CYCLE CAP / SAFE_IDLE ----
        if (recovery_cycle_count_ >= global_cycle_cap_)
        {
            recovery_state_ = RecoveryState::SAFE_IDLE;
            { std::lock_guard<std::mutex> lk(traj_mutex_); has_traj_ = false; }
            clearRotate();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "[recovery] SAFE_IDLE: exhausted %d cycles, no path. Holding, awaiting "
                "new goal/global_path/map change.", recovery_cycle_count_);
            return;
        }

        // Nothing swapped this tick and no rotate/backoff active: hold position.
        // SINGLE terminal fallback (replaces the old three bare freeze exits).
        { std::lock_guard<std::mutex> lk(traj_mutex_); has_traj_ = false; }
    }

    // Synthesize + swap a short retreat trajectory along increasing clearance.
    bool doBackoff(const Eigen::Vector2d &cur_pos, const Eigen::Vector2d &cur_vel,
                   double cur_yaw)
    {
        double dist;
        Eigen::Vector2d grad;
        edt_env_->evaluateEDTWithGrad(cur_pos, -1.0, dist, grad);  // grad -> more clearance

        Eigen::Vector2d dir;
        if (grad.norm() > 1e-3)         dir = grad.normalized();
        else if (cur_vel.norm() > 1e-3) dir = -cur_vel.normalized();
        else                            dir = -Eigen::Vector2d(std::cos(cur_yaw), std::sin(cur_yaw));

        // 180-deg flip debounce: refuse a retreat that reverses the last one
        // (prevents a forward/back wedge limit-cycle); escalate to rotate instead.
        if (last_recovery_dir_.norm() > 1e-3 && dir.dot(last_recovery_dir_) < -0.7)
            return false;

        // Size the retreat so the target improves clearance and stays in map;
        // shrink if the full step would not.
        Eigen::Vector2d target;
        bool found = false;
        for (double f = 1.0; f >= 0.25; f -= 0.25)
        {
            Eigen::Vector2d t = cur_pos + dir * (backoff_distance_ * f);
            if (edt_env_->isInMap(t) && edt_env_->getDistance(t) > edt_env_->getDistance(cur_pos))
            {
                target = t;
                found = true;
                break;
            }
        }
        if (!found) return false;   // no improving retreat exists -> caller escalates to rotate

        double max_vel = 3.0, max_acc = 3.0;
        this->get_parameter("search.max_vel", max_vel);
        this->get_parameter("search.max_acc", max_acc);

        auto rt = std::make_shared<sentry_planner::MincoTrajectory>(minco_template_);
        std::vector<Eigen::Vector2d> wp = {cur_pos, target};
        rt->setup(wp, cur_vel, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
                  Eigen::Vector2d::Zero(), max_vel * backoff_max_vel_scale_, max_acc,
                  this->now().seconds());
        if (rt->empty() || rt->getDuration() < 1e-3) return false;

        const double d = rt->getDuration();
        const rclcpp::Time s = this->now();
        {
            std::lock_guard<std::mutex> lk(traj_mutex_);
            active_traj_     = rt;
            traj_duration_   = d;
            traj_start_time_ = s;
            has_traj_        = true;
        }
        publishTrajectory(*rt, d);
        last_recovery_dir_ = dir;
        recovery_last_clearance_ = edt_env_->getDistance(target);  // for progress-based cap reset
        clearRotate();   // backoff supersedes any rotate
        return true;
    }

    void startRotate(double target_bearing, double cur_yaw)
    {
        double err = target_bearing - cur_yaw;
        while (err > M_PI)  err -= 2.0 * M_PI;
        while (err < -M_PI) err += 2.0 * M_PI;
        double rate = (err >= 0 ? +1.0 : -1.0) * rotate_yaw_rate_;
        rotate_deadline_ = this->now() + rclcpp::Duration::from_seconds(rotate_max_time_);
        {
            std::lock_guard<std::mutex> lk(recovery_mutex_);
            recovery_rotate_active_ = true;
            recovery_rotate_rate_   = rate;
        }
        // rotate uses the no-traj branch; clear the traj (separate, non-nested lock)
        { std::lock_guard<std::mutex> lk(traj_mutex_); has_traj_ = false; }
    }

    void clearRotate()
    {
        std::lock_guard<std::mutex> lk(recovery_mutex_);
        recovery_rotate_active_ = false;
        recovery_rotate_rate_   = 0.0;
    }

    // Full reset of recovery state (new goal / new global path). DEFAULT-group only.
    void resetRecovery()
    {
        recovery_state_       = RecoveryState::NORMAL;
        recovery_fail_count_  = 0;
        recovery_cycle_count_ = 0;
        backoff_cooldown_     = 0;
        recovery_best_clearance_ = -1.0;
        last_recovery_dir_.setZero();
        clearRotate();
    }

    void replanCallback()
    {
        Eigen::Vector2d goal, cur_pos;
        bool has_odom, has_goal;
        {
            std::lock_guard<std::mutex> lk(state_mutex_);
            has_odom = has_odom_;
            has_goal = has_final_goal_;
            goal     = final_goal_;
            cur_pos  = current_pos_;
        }

        if (!has_odom || !has_goal) return;

        // --- Recovery bookkeeping (single source of truth for cooldown ageing) ---
        // Must run BEFORE the near-goal early-out: an in-place ROTATE that begins
        // within 1 m of the goal would otherwise never reach its deadline and spin
        // forever. backoff_cooldown_ is decremented exactly once per replan tick here.
        if (backoff_cooldown_ > 0) backoff_cooldown_--;
        // Time-box ROTATE: once the deadline passes, drop the spin and re-attempt
        // a reroute from the (now-changed) heading.
        if (recovery_state_ == RecoveryState::ROTATE && this->now() > rotate_deadline_)
        {
            clearRotate();
            recovery_state_ = RecoveryState::REROUTE;
        }

        if ((goal - cur_pos).norm() < 1.0) return;

        Eigen::Vector2d local_goal = goal;
        if (has_global_path_ && !global_waypoints_.empty())
        {
            while (current_waypoint_idx_ < (int)global_waypoints_.size() - 1 &&
                   (global_waypoints_[current_waypoint_idx_] - cur_pos).norm() < 1.0)
                current_waypoint_idx_++;
            // SKIP-WAYPOINT reroute: when already in REROUTE, advance to the next
            // global waypoint (monotonic forward only) so the global path's own
            // alternative takes over before we exhaust the local fan again.
            if (recovery_state_ == RecoveryState::REROUTE &&
                current_waypoint_idx_ < (int)global_waypoints_.size() - 1)
                current_waypoint_idx_++;
            local_goal = global_waypoints_[current_waypoint_idx_];
        }
        planToGoal(local_goal);
    }

    void controlLoop()
    {
        // Snapshot INPUT state under state_mutex_ (shared across callback groups);
        // do all subsequent work on these locals. state_mutex_ is released before
        // traj_mutex_ is ever taken below — the two are never held together.
        Eigen::Vector2d cur_pos, cur_vel, goal;
        double cur_yaw = 0.0;
        bool has_odom = false, has_goal = false;
        {
            std::lock_guard<std::mutex> lk(state_mutex_);
            has_odom = has_odom_;
            cur_pos  = current_pos_;
            cur_vel  = current_vel_;
            cur_yaw  = current_yaw_;
            goal     = final_goal_;
            has_goal = has_final_goal_;
        }
        if (!has_odom) return;

        // Snapshot trajectory state under lock; do all subsequent work on the
        // local copies so a replan mid-control-cycle can't tear the state.
        std::shared_ptr<sentry_planner::MincoTrajectory> snap;
        double dur = 0.0;
        rclcpp::Time start;
        bool has = false;
        {
            std::lock_guard<std::mutex> lk(traj_mutex_);
            snap  = active_traj_;
            dur   = traj_duration_;
            start = traj_start_time_;
            has   = has_traj_;
        }

        double elapsed = has ? (this->now() - start).seconds() : 0.0;

        if (has && elapsed >= dur)
        {
            {
                std::lock_guard<std::mutex> lk(traj_mutex_);
                has_traj_ = false;
            }
            has = false;
            if (!has_goal || (goal - cur_pos).norm() < 1.0) {
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                std::lock_guard<std::mutex> lk(state_mutex_);
                has_final_goal_ = false;
            }
        }

        // Recovery ROTATE: with no active trajectory, an in-place re-aim spin is
        // the only non-trajectory recovery motion. Snapshot the rotate field under
        // recovery_mutex_ alone (state_mutex_/traj_mutex_ already released above —
        // three sequential locks, never nested).
        if (!has)
        {
            bool rot = false;
            double rot_rate = 0.0;
            {
                std::lock_guard<std::mutex> lk(recovery_mutex_);
                rot      = recovery_rotate_active_;
                rot_rate = recovery_rotate_rate_;
            }
            if (rot)
            {
                geometry_msgs::msg::Twist cmd;   // zero linear
                cmd.angular.z = tracker_.limitYawRate(rot_rate);
                cmd_vel_pub_->publish(cmd);
                return;
            }
        }

        static const sentry_planner::MincoTrajectory empty_traj;
        const auto &traj_ref = (has && snap) ? *snap : empty_traj;
        auto cmd = tracker_.compute(cur_pos, cur_vel, cur_yaw,
                                     traj_ref, elapsed, has && snap);
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
    // Holds the optimizer config set once at init; replans copy from it.
    sentry_planner::MincoTrajectory minco_template_;
    sentry_planner::TrajectoryTracker tracker_;

    // State
    Eigen::Vector2d current_pos_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d current_vel_ = Eigen::Vector2d::Zero();
    double current_yaw_ = 0.0;
    bool has_odom_ = false;
    bool has_velocity_estimate_ = false;
    Eigen::Vector2d smoothed_vel_ = Eigen::Vector2d::Zero();  // control-group-local velocity accumulator
    Eigen::Vector2d last_odom_pos_ = Eigen::Vector2d::Zero();
    rclcpp::Time last_odom_stamp_;
    bool has_last_odom_ = false;

    // Guards INPUT state shared across callback groups: current_pos_, current_vel_,
    // current_yaw_, has_odom_, final_goal_, has_final_goal_. Never held together
    // with traj_mutex_ (no nesting; snapshot-then-release everywhere).
    mutable std::mutex state_mutex_;

    // Trajectory state shared between replan (writer) and control (reader).
    // All four fields below are guarded by traj_mutex_.
    mutable std::mutex traj_mutex_;
    std::shared_ptr<sentry_planner::MincoTrajectory> active_traj_;
    double traj_duration_ = 0.0;
    rclcpp::Time traj_start_time_;
    bool has_traj_ = false;

    Eigen::Vector2d final_goal_ = Eigen::Vector2d::Zero();
    bool has_final_goal_ = false;

    std::vector<Eigen::Vector2d> global_waypoints_;
    int current_waypoint_idx_ = 0;
    bool has_global_path_ = false;

    // --- Recovery state (DEFAULT-group-only, NO mutex) ---
    // planToGoal/planFailed are reached only from replanCallback and goalCallback,
    // both in the DEFAULT MutuallyExclusive group (same as global_waypoints_), so
    // these are serialised against themselves and never touched by controlLoop.
    double robot_radius_ = 0.3;
    std::vector<double> fan_offsets_deg_;
    std::vector<double> fan_radius_scales_;
    int    max_astar_attempts_     = 8;
    double cand_clearance_margin_  = 0.1;
    double backoff_distance_       = 0.45;
    double backoff_max_vel_scale_  = 0.5;
    int    backoff_cooldown_cycles_ = 2;
    bool   rotate_enable_          = true;
    double rotate_yaw_rate_        = 0.6;
    double rotate_max_time_        = 1.5;
    int    global_cycle_cap_       = 5;

    RecoveryState recovery_state_  = RecoveryState::NORMAL;
    int recovery_fail_count_       = 0;   // consecutive ticks where even the fan found nothing
    int recovery_cycle_count_      = 0;   // full ladder cycles, for global_cycle_cap_
    int backoff_cooldown_          = 0;   // >0 suppresses a repeat backoff this many ticks
    double recovery_best_clearance_ = -1.0; // best ESDF clearance reached this recovery episode
    double recovery_last_clearance_ = 0.0;  // clearance at the most recent committed backoff target
    Eigen::Vector2d last_recovery_dir_ = Eigen::Vector2d::Zero();  // last backoff dir (flip debounce)
    rclcpp::Time rotate_deadline_;        // DEFAULT-group-only

    // The ONE new mutex: guards ONLY the two rotate fields below. Written by the
    // DEFAULT group (startRotate/clearRotate), read by controlLoop (CONTROL group).
    // NEVER held together with state_mutex_ or traj_mutex_.
    std::mutex recovery_mutex_;
    bool   recovery_rotate_active_ = false;
    double recovery_rotate_rate_   = 0.0;  // signed rad/s, toward target bearing

    // ROS
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr ctrl_timer_, replan_timer_;
    rclcpp::CallbackGroup::SharedPtr control_cb_group_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SentryLocalPlannerNode>();
    node->initialize();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
