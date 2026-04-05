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

#include <sentry_planner/minco_trajectory.hpp>
#include <sentry_planner/mpc_controller.hpp>

using namespace fast_planner;

class SentryPlannerNode : public rclcpp::Node
{
public:
    SentryPlannerNode() : Node("sentry_planner")
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
            std::bind(&SentryPlannerNode::goalCallback, this, std::placeholders::_1));

        // --- Publishers ---
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/trajectory", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/planning/trajectory_markers", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // --- Controller timer ---
        ctrl_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / ctrl_freq),
            std::bind(&SentryPlannerNode::controlLoop, this));

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

            minco_traj_.setup(point_set, start_vel, Eigen::Vector2d::Zero(),
                              end_vel, Eigen::Vector2d::Zero(), max_vel, max_acc);
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

    // ==================== Control loop ====================
    void controlLoop()
    {
        if (!has_traj_ || !has_odom_)
            return;

        double elapsed = (this->now() - traj_start_time_).seconds();

        if (elapsed >= traj_duration_)
        {
            // Stop
            cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
            has_traj_ = false;

            // Re-plan toward final goal?
            if (has_final_goal_ && (final_goal_ - current_pos_).norm() > 0.3) {
                RCLCPP_INFO(this->get_logger(), "Segment done, %.1fm left, re-planning...",
                    (final_goal_ - current_pos_).norm());
                planToGoal(final_goal_);
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                has_final_goal_ = false;
            }
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

    // ==================== ROS ====================
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr ctrl_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SentryPlannerNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
