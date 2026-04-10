/**
 * sentry_planner_node: 集成规划 + 控制, 支持双模式切换
 *
 * 模式A (bspline+pd):  Kinodynamic A* → B-spline 优化 → PD 前馈跟踪
 * 模式B (minco+mpc):   Kinodynamic A* → MINCO 最小 jerk → MPC 跟踪
 *
 * 通过 planner_mode / controller_mode 参数切换
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <plan_env/sdf_map.hpp>
#include <plan_env/edt_environment.hpp>
#include <path_searching/kinodynamic_astar.hpp>
#include <bspline/non_uniform_bspline.hpp>
#include <bspline_opt/bspline_optimizer.hpp>

#include <sentry_local_planner/minco_trajectory.hpp>
#include <sentry_local_planner/mpc_controller.hpp>

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

        // --- Init plan_env ---
        sdf_map_ = std::make_shared<SDFMap>();
        sdf_map_->initMap(node_ptr);
        edt_env_ = std::make_shared<EDTEnvironment>();
        edt_env_->setMap(sdf_map_);

        // --- Init A* (共用前端) ---
        kino_astar_ = std::make_shared<KinodynamicAstar>();
        kino_astar_->setParam(node_ptr);
        kino_astar_->setEnvironment(edt_env_);
        kino_astar_->init();

        // --- Init B-spline optimizer (模式A) ---
        bspline_opt_ = std::allocate_shared<BsplineOptimizer>(
            Eigen::aligned_allocator<BsplineOptimizer>());
        bspline_opt_->setParam(node_ptr);
        bspline_opt_->setEnvironment(edt_env_);

        // --- Init MINCO optimizer (模式B) ---
        this->declare_parameter<double>("minco_opt.lambda_smooth", 0.1);
        this->declare_parameter<double>("minco_opt.lambda_col",    8.0);
        this->declare_parameter<double>("minco_opt.lambda_feas",   0.001);
        this->declare_parameter<double>("minco_opt.dist0",         0.05);
        this->declare_parameter<double>("minco_opt.robot_radius",  0.3);
        this->declare_parameter<int>   ("minco_opt.num_samples",   8);
        this->declare_parameter<int>   ("minco_opt.max_iter",      200);
        this->declare_parameter<double>("minco_opt.max_time_ms",   20.0);
        {
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
        }

        // --- Mode selection ---
        this->declare_parameter<std::string>("planner_mode", "minco");    // "bspline" | "minco"
        this->declare_parameter<std::string>("controller_mode", "mpc");   // "pd" | "mpc"

        planner_mode_ = this->get_parameter("planner_mode").as_string();
        controller_mode_ = this->get_parameter("controller_mode").as_string();

        // --- Controller parameters ---
        this->declare_parameter<double>("controller.frequency", 50.0);
        this->declare_parameter<double>("controller.kp", 2.0);
        this->declare_parameter<int>("mpc.horizon", 10);
        this->declare_parameter<double>("mpc.q_pos", 10.0);
        this->declare_parameter<double>("mpc.q_vel", 1.0);
        this->declare_parameter<double>("mpc.r_acc", 0.1);

        double ctrl_freq = this->get_parameter("controller.frequency").as_double();
        config_dt_ = 1.0 / ctrl_freq;
        kp_ = this->get_parameter("controller.kp").as_double();

        // --- Init MPC ---
        sentry_planner::MPCController::Config mpc_cfg;
        mpc_cfg.horizon = this->get_parameter("mpc.horizon").as_int();
        mpc_cfg.dt = 1.0 / ctrl_freq;
        mpc_cfg.q_pos = this->get_parameter("mpc.q_pos").as_double();
        mpc_cfg.q_vel = this->get_parameter("mpc.q_vel").as_double();
        mpc_cfg.r_acc = this->get_parameter("mpc.r_acc").as_double();
        this->get_parameter("search.max_vel", mpc_cfg.max_vel);
        this->get_parameter("search.max_acc", mpc_cfg.max_acc);
        mpc_.setConfig(mpc_cfg);

        // --- Replan safety check ---
        // (reactive parameters kept for reference but 10Hz proactive replan is primary)

        robot_radius_ = this->get_parameter("minco_opt.robot_radius").as_double();
        footprint_offsets_ = {
            Eigen::Vector2d(0, 0),
            Eigen::Vector2d( robot_radius_, 0),
            Eigen::Vector2d(-robot_radius_, 0),
            Eigen::Vector2d(0,  robot_radius_),
            Eigen::Vector2d(0, -robot_radius_),
        };

        // --- Spin (self-rotation for better LiDAR coverage) ---
        this->declare_parameter<double>("controller.spin_rate", 3.0);
        this->declare_parameter<double>("controller.narrow_passage_dist", 0.5);
        this->declare_parameter<double>("controller.yaw_align_kp", 3.0);
        spin_rate_ = this->get_parameter("controller.spin_rate").as_double();
        narrow_passage_dist_ = this->get_parameter("controller.narrow_passage_dist").as_double();
        yaw_align_kp_ = this->get_parameter("controller.yaw_align_kp").as_double();

        // --- Subscribers ---
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                current_pos_(0) = msg->pose.pose.position.x;
                current_pos_(1) = msg->pose.pose.position.y;
                current_vel_(0) = msg->twist.twist.linear.x;
                current_vel_(1) = msg->twist.twist.linear.y;
                double qw = msg->pose.pose.orientation.w;
                double qz = msg->pose.pose.orientation.z;
                current_yaw_ = 2.0 * atan2(qz, qw);
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
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // --- Controller timer ---
        ctrl_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / ctrl_freq),
            std::bind(&SentryLocalPlannerNode::controlLoop, this));

        // --- 10Hz proactive replan timer ---
        this->declare_parameter<double>("replan.frequency", 10.0);
        double replan_freq = this->get_parameter("replan.frequency").as_double();
        replan_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / replan_freq),
            std::bind(&SentryLocalPlannerNode::replanCallback, this));

        RCLCPP_INFO(this->get_logger(),
            "Planner initialized: mode=%s+%s, ctrl@%.0fHz",
            planner_mode_.c_str(), controller_mode_.c_str(), ctrl_freq);
    }

private:
    // ==================== Goal handling ====================
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!has_odom_) {
            RCLCPP_WARN(this->get_logger(), "No odom yet");
            return;
        }
        final_goal_ = Eigen::Vector2d(msg->pose.position.x, msg->pose.position.y);
        has_final_goal_ = true;
        planToGoal(final_goal_);
    }

    void planToGoal(Eigen::Vector2d goal_pt)
    {
        // Clamp to local ESDF range
        Eigen::Vector2d diff = goal_pt - current_pos_;

        // Already at goal — no need to re-plan
        const double goal_tolerance = 0.4;
        if (diff.norm() < goal_tolerance) {
            RCLCPP_INFO(this->get_logger(), "Already within %.2fm of goal, skipping plan", goal_tolerance);
            return;
        }

        double local_range = 7.5;
        this->get_parameter("sdf_map.local_update_range_x", local_range);
        local_range -= 0.5;
        bool clamped = false;
        if (diff.norm() > local_range) {
            goal_pt = current_pos_ + diff.normalized() * local_range;
            clamped = true;
        }

        // Clamp to map boundary
        Eigen::Vector2d map_origin, map_size;
        sdf_map_->getRegion(map_origin, map_size);
        for (int i = 0; i < 2; ++i) {
            goal_pt(i) = std::clamp(goal_pt(i), map_origin(i) + 0.5, map_origin(i) + map_size(i) - 0.5);
        }

        RCLCPP_INFO(this->get_logger(), "[%s+%s] Plan: (%.2f,%.2f)->(%.2f,%.2f)%s",
            planner_mode_.c_str(), controller_mode_.c_str(),
            current_pos_(0), current_pos_(1), goal_pt(0), goal_pt(1),
            clamped ? " [clamped]" : "");

        // --- A* search (共用) ---
        kino_astar_->reset();
        int result = kino_astar_->search(
            current_pos_, current_vel_, Eigen::Vector2d::Zero(),
            goal_pt, Eigen::Vector2d::Zero(), true);

        if (result == KinodynamicAstar::NO_PATH) {
            RCLCPP_WARN(this->get_logger(), "No path found!");
            has_traj_ = false;
            return;
        }

        double ts;
        std::vector<Eigen::Vector2d> point_set, start_end_derivatives;
        kino_astar_->getSamples(ts, point_set, start_end_derivatives);

        if (point_set.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Too few A* samples");
            has_traj_ = false;
            return;
        }

        // --- 后端: B-spline 或 MINCO ---
        double max_vel = 3.0, max_acc = 3.0;
        this->get_parameter("search.max_vel", max_vel);
        this->get_parameter("search.max_acc", max_acc);

        if (planner_mode_ == "minco")
        {
            Eigen::Vector2d start_vel = (start_end_derivatives.size() >= 2) ? start_end_derivatives[0] : current_vel_;
            Eigen::Vector2d end_vel = (start_end_derivatives.size() >= 2) ? start_end_derivatives[1] : Eigen::Vector2d::Zero();
            Eigen::Vector2d start_acc = (start_end_derivatives.size() >= 4) ? start_end_derivatives[2] : Eigen::Vector2d::Zero();
            Eigen::Vector2d end_acc = (start_end_derivatives.size() >= 4) ? start_end_derivatives[3] : Eigen::Vector2d::Zero();

            minco_traj_.setup(point_set, start_vel, start_acc,
                              end_vel, end_acc, max_vel, max_acc,
                              this->now().seconds());
            traj_duration_ = minco_traj_.getDuration();
            use_minco_ = true;
        }
        else // bspline
        {
            Eigen::MatrixXd ctrl_pts;
            NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
            Eigen::MatrixXd opt_ctrl_pts = bspline_opt_->BsplineOptimizeTraj(
                ctrl_pts, ts, BsplineOptimizer::NORMAL_PHASE, 1, 1);

            bspline_traj_ = NonUniformBspline(opt_ctrl_pts, 3, ts);
            bspline_traj_.setPhysicalLimits(max_vel, max_acc);
            bspline_traj_.reallocateTime();
            bspline_vel_ = bspline_traj_.getDerivative();
            traj_duration_ = bspline_traj_.getTimeSum();
            use_minco_ = false;
        }

        traj_start_time_ = this->now();
        has_traj_ = true;

        // Publish visualization
        publishTrajectory();

        RCLCPP_INFO(this->get_logger(), "Trajectory: duration=%.2fs, %zu waypoints",
                     traj_duration_, point_set.size());
    }

    // ==================== 10Hz proactive replan ====================
    void replanCallback()
    {
        if (!has_odom_ || !has_final_goal_)
            return;

        // 已到达最终目标附近，不重规划
        if ((final_goal_ - current_pos_).norm() < 0.4)
            return;

        Eigen::Vector2d local_goal = final_goal_;

        if (has_global_path_ && !global_waypoints_.empty())
        {
            // 前进 waypoint: 跳过已经过的
            while (current_waypoint_idx_ < (int)global_waypoints_.size() - 1 &&
                   (global_waypoints_[current_waypoint_idx_] - current_pos_).norm() < 1.0)
                current_waypoint_idx_++;

            local_goal = global_waypoints_[current_waypoint_idx_];
        }

        planToGoal(local_goal);
    }

    // ==================== Spin / narrow-passage alignment ====================
    double computeAngularVelocity(double elapsed)
    {
        // 检查当前位置附近 ESDF 距离
        if (sdf_map_->isInMap(current_pos_))
        {
            double center_dist = sdf_map_->getDistance(current_pos_);
            if (center_dist < narrow_passage_dist_)
            {
                // 窄口: 停止自转，对齐底盘长轴与轨迹速度方向
                Eigen::Vector2d vel_dir;
                if (use_minco_)
                    vel_dir = minco_traj_.getVelocity(elapsed);
                else
                    vel_dir = bspline_vel_.evaluateDeBoorT(elapsed).head(2);

                if (vel_dir.norm() > 0.3)
                {
                    double desired_yaw = atan2(vel_dir(1), vel_dir(0));
                    double yaw_err = desired_yaw - current_yaw_;
                    // 归一化到 [-π, π]
                    while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
                    while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

                    // 底盘对称: 转 180° 也是同样的截面，选最近的
                    if (yaw_err > M_PI / 2.0) yaw_err -= M_PI;
                    else if (yaw_err < -M_PI / 2.0) yaw_err += M_PI;

                    return yaw_align_kp_ * yaw_err;
                }
                return 0.0;  // 速度太小，不转
            }
        }
        return spin_rate_;  // 正常自转
    }

    // ==================== Control loop ====================
    void controlLoop()
    {
        if (!has_odom_)
            return;

        // 无轨迹时仍保持自转
        if (!has_traj_)
        {
            if (spin_rate_ != 0.0)
            {
                geometry_msgs::msg::Twist cmd;
                cmd.angular.z = spin_rate_;
                cmd_vel_pub_->publish(cmd);
            }
            return;
        }

        double elapsed = (this->now() - traj_start_time_).seconds();

        if (elapsed >= traj_duration_)
        {
            // 轨迹结束，停止平移，保持自转
            geometry_msgs::msg::Twist stop_cmd;
            stop_cmd.angular.z = spin_rate_;
            cmd_vel_pub_->publish(stop_cmd);
            has_traj_ = false;

            if (!has_final_goal_ || (final_goal_ - current_pos_).norm() < 0.4) {
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                has_final_goal_ = false;
            }
            // 若还有目标，replanCallback 会在下次 10Hz tick 自动重规划
            return;
        }

        Eigen::Vector2d vel_cmd_odom;

        if (controller_mode_ == "mpc")
        {
            // --- MPC 双积分器: 输出加速度 ---
            auto ref_func = [this](double t) -> Eigen::Vector4d {
                Eigen::Vector4d ref;
                if (use_minco_) {
                    ref.head(2) = minco_traj_.getPosition(t);
                    ref.tail(2) = minco_traj_.getVelocity(t);
                } else {
                    ref.head(2) = bspline_traj_.evaluateDeBoorT(t).head(2);
                    ref.tail(2) = bspline_vel_.evaluateDeBoorT(t).head(2);
                }
                return ref;
            };

            Eigen::Vector2d acc_cmd = mpc_.compute(current_pos_, current_vel_, ref_func, elapsed);

            // 参考速度前馈 + MPC 加速度积分
            Eigen::Vector2d vel_ref;
            if (use_minco_)
                vel_ref = minco_traj_.getVelocity(elapsed);
            else
                vel_ref = bspline_vel_.evaluateDeBoorT(elapsed).head(2);

            // v_cmd = v_ref + a_mpc * dt_lookahead
            // dt_lookahead 用 MPC horizon 时间的一部分，让加速度修正更明显
            double dt_lookahead = config_dt_ * 3.0;
            vel_cmd_odom = vel_ref + acc_cmd * dt_lookahead;
        }
        else
        {
            // --- PD 控制 ---
            Eigen::Vector2d pos_d, vel_d;
            if (use_minco_) {
                pos_d = minco_traj_.getPosition(elapsed);
                vel_d = minco_traj_.getVelocity(elapsed);
            } else {
                pos_d = bspline_traj_.evaluateDeBoorT(elapsed).head(2);
                vel_d = bspline_vel_.evaluateDeBoorT(elapsed).head(2);
            }

            Eigen::Vector2d pos_err = pos_d - current_pos_;
            vel_cmd_odom = vel_d + kp_ * pos_err;
        }

        // 限速
        double max_vel = 3.0;
        this->get_parameter("search.max_vel", max_vel);
        if (vel_cmd_odom.norm() > max_vel)
            vel_cmd_odom = vel_cmd_odom.normalized() * max_vel;

        // Odom → body frame
        double cy = cos(-current_yaw_), sy = sin(-current_yaw_);
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = cy * vel_cmd_odom(0) - sy * vel_cmd_odom(1);
        cmd.linear.y = sy * vel_cmd_odom(0) + cy * vel_cmd_odom(1);

        // 自转 vs 窄口对齐
        cmd.angular.z = computeAngularVelocity(elapsed);
        cmd_vel_pub_->publish(cmd);
    }

    // ==================== Visualization ====================
    void publishTrajectory()
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
        line.ns = "trajectory";
        line.id = 0;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.08;
        // 颜色区分模式: 绿色=bspline, 青色=minco
        line.color.r = use_minco_ ? 0.0 : 0.0;
        line.color.g = 1.0;
        line.color.b = use_minco_ ? 1.0 : 0.0;
        line.color.a = 1.0;

        double dt = 0.05;
        for (double t = 0; t <= traj_duration_; t += dt)
        {
            Eigen::Vector2d pt;
            if (use_minco_)
                pt = minco_traj_.getPosition(t);
            else
                pt = bspline_traj_.evaluateDeBoorT(t).head(2);

            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = pt(0);
            pose.pose.position.y = pt(1);
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

    // ==================== Components ====================
    std::shared_ptr<SDFMap> sdf_map_;
    std::shared_ptr<EDTEnvironment> edt_env_;
    std::shared_ptr<KinodynamicAstar> kino_astar_;
    std::shared_ptr<BsplineOptimizer> bspline_opt_;

    // ==================== Mode ====================
    std::string planner_mode_;    // "bspline" | "minco"
    std::string controller_mode_; // "pd" | "mpc"

    // ==================== Trajectory backends ====================
    bool use_minco_ = false;
    // B-spline
    NonUniformBspline bspline_traj_;
    NonUniformBspline bspline_vel_;
    // MINCO
    sentry_planner::MincoTrajectory minco_traj_;

    // ==================== Controller backends ====================
    double kp_ = 2.0;                       // PD gain
    double config_dt_ = 0.02;               // 控制周期
    double spin_rate_ = 3.0;                // 自转角速度 (rad/s)
    double narrow_passage_dist_ = 0.5;      // ESDF < 此值时切换为航向对齐
    double yaw_align_kp_ = 3.0;             // 航向对齐 P 增益
    sentry_planner::MPCController mpc_;      // MPC

    // ==================== State ====================
    Eigen::Vector2d current_pos_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d current_vel_ = Eigen::Vector2d::Zero();
    double current_yaw_ = 0.0;
    bool has_odom_ = false;

    double traj_duration_ = 0.0;
    rclcpp::Time traj_start_time_;
    bool has_traj_ = false;

    Eigen::Vector2d final_goal_ = Eigen::Vector2d::Zero();
    bool has_final_goal_ = false;

    // Replan
    double robot_radius_ = 0.3;
    std::vector<Eigen::Vector2d> footprint_offsets_;

    // Global path tracking
    std::vector<Eigen::Vector2d> global_waypoints_;
    int current_waypoint_idx_ = 0;
    bool has_global_path_ = false;

    // ==================== ROS ====================
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr ctrl_timer_;
    rclcpp::TimerBase::SharedPtr replan_timer_;
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
