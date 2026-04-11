/**
 * sentry_local_planner_node: Kinodynamic A* + MINCO + MPC
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
        tracker_.init(tcfg, edt_env_.get());

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

        // --- Timers ---
        ctrl_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / ctrl_freq),
            std::bind(&SentryLocalPlannerNode::controlLoop, this));

        this->declare_parameter<double>("replan.frequency", 10.0);
        double replan_freq = this->get_parameter("replan.frequency").as_double();
        replan_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / replan_freq),
            std::bind(&SentryLocalPlannerNode::replanCallback, this));

        RCLCPP_INFO(this->get_logger(), "Planner initialized: minco+mpc, ctrl@%.0fHz", ctrl_freq);
    }

private:
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!has_odom_) { RCLCPP_WARN(this->get_logger(), "No odom yet"); return; }
        final_goal_ = Eigen::Vector2d(msg->pose.position.x, msg->pose.position.y);
        has_final_goal_ = true;
        planToGoal(final_goal_);
    }

    void planToGoal(Eigen::Vector2d goal_pt)
    {
        Eigen::Vector2d diff = goal_pt - current_pos_;
        if (diff.norm() < 0.4) return;

        double local_range = 7.5;
        this->get_parameter("sdf_map.local_update_range_x", local_range);
        local_range -= 0.5;
        if (diff.norm() > local_range)
            goal_pt = current_pos_ + diff.normalized() * local_range;

        Eigen::Vector2d map_origin, map_size;
        sdf_map_->getRegion(map_origin, map_size);
        for (int i = 0; i < 2; ++i)
            goal_pt(i) = std::clamp(goal_pt(i), map_origin(i) + 0.5, map_origin(i) + map_size(i) - 0.5);

        // A*
        kino_astar_->reset();
        int result = kino_astar_->search(current_pos_, current_vel_, Eigen::Vector2d::Zero(),
                                          goal_pt, Eigen::Vector2d::Zero(), true);
        if (result == KinodynamicAstar::NO_PATH) { has_traj_ = false; return; }

        double ts;
        std::vector<Eigen::Vector2d> point_set, start_end_derivatives;
        kino_astar_->getSamples(ts, point_set, start_end_derivatives);
        if (point_set.size() < 2) { has_traj_ = false; return; }

        // MINCO
        double max_vel = 3.0, max_acc = 3.0;
        this->get_parameter("search.max_vel", max_vel);
        this->get_parameter("search.max_acc", max_acc);

        Eigen::Vector2d sv = start_end_derivatives.size() >= 2 ? start_end_derivatives[0] : current_vel_;
        Eigen::Vector2d ev = start_end_derivatives.size() >= 2 ? start_end_derivatives[1] : Eigen::Vector2d::Zero();
        Eigen::Vector2d sa = start_end_derivatives.size() >= 4 ? start_end_derivatives[2] : Eigen::Vector2d::Zero();
        Eigen::Vector2d ea = start_end_derivatives.size() >= 4 ? start_end_derivatives[3] : Eigen::Vector2d::Zero();

        minco_traj_.setup(point_set, sv, sa, ev, ea, max_vel, max_acc, this->now().seconds());
        traj_duration_ = minco_traj_.getDuration();
        traj_start_time_ = this->now();
        has_traj_ = true;
        publishTrajectory();
    }

    void replanCallback()
    {
        if (!has_odom_ || !has_final_goal_) return;
        if ((final_goal_ - current_pos_).norm() < 0.4) return;

        Eigen::Vector2d local_goal = final_goal_;
        if (has_global_path_ && !global_waypoints_.empty())
        {
            while (current_waypoint_idx_ < (int)global_waypoints_.size() - 1 &&
                   (global_waypoints_[current_waypoint_idx_] - current_pos_).norm() < 1.0)
                current_waypoint_idx_++;
            local_goal = global_waypoints_[current_waypoint_idx_];
        }
        planToGoal(local_goal);
    }

    void controlLoop()
    {
        if (!has_odom_) return;

        double elapsed = has_traj_ ? (this->now() - traj_start_time_).seconds() : 0.0;

        if (has_traj_ && elapsed >= traj_duration_)
        {
            has_traj_ = false;
            if (!has_final_goal_ || (final_goal_ - current_pos_).norm() < 0.4) {
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                has_final_goal_ = false;
            }
        }

        auto cmd = tracker_.compute(current_pos_, current_vel_, current_yaw_,
                                     minco_traj_, elapsed, has_traj_);
        cmd_vel_pub_->publish(cmd);
    }

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
        line.ns = "trajectory"; line.id = 0;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.08;
        line.color.g = 1.0; line.color.b = 1.0; line.color.a = 1.0;

        for (double t = 0; t <= traj_duration_; t += 0.05)
        {
            Eigen::Vector2d pt = minco_traj_.getPosition(t);
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
    sentry_planner::MincoTrajectory minco_traj_;
    sentry_planner::TrajectoryTracker tracker_;

    // State
    Eigen::Vector2d current_pos_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d current_vel_ = Eigen::Vector2d::Zero();
    double current_yaw_ = 0.0;
    bool has_odom_ = false;

    double traj_duration_ = 0.0;
    rclcpp::Time traj_start_time_;
    bool has_traj_ = false;

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
