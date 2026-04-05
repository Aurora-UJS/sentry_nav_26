/**
 * sentry_planner_node: 集成 plan_env + KinodynamicAstar + BSpline + Controller
 *
 * 订阅: /Odometry (位姿), /cloud_registered (点云), /goal_pose (目标点)
 * 发布: /planning/trajectory (nav_msgs/Path), /planning/trajectory_markers (可视化)
 *       /sdf_map/occupancy, /sdf_map/esdf (由 plan_env 内部发布)
 *       /cmd_vel (geometry_msgs/Twist, 轨迹跟踪输出)
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

using namespace fast_planner;

class SentryPlannerNode : public rclcpp::Node
{
public:
    SentryPlannerNode() : Node("sentry_planner")
    {
        // Declare manager.odometry param before plan_env needs it
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

        // --- Init path searcher ---
        kino_astar_ = std::make_shared<KinodynamicAstar>();
        kino_astar_->setParam(node_ptr);
        kino_astar_->setEnvironment(edt_env_);
        kino_astar_->init();

        // --- Init bspline optimizer ---
        bspline_opt_ = std::allocate_shared<BsplineOptimizer>(
            Eigen::aligned_allocator<BsplineOptimizer>());
        bspline_opt_->setParam(node_ptr);
        bspline_opt_->setEnvironment(edt_env_);

        // --- Controller parameters ---
        this->declare_parameter<double>("controller.frequency", 50.0);
        this->declare_parameter<double>("controller.kp", 2.0);
        double ctrl_freq = this->get_parameter("controller.frequency").as_double();
        kp_ = this->get_parameter("controller.kp").as_double();

        // --- Subscribers ---
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                current_pos_(0) = msg->pose.pose.position.x;
                current_pos_(1) = msg->pose.pose.position.y;
                current_vel_(0) = msg->twist.twist.linear.x;
                current_vel_(1) = msg->twist.twist.linear.y;
                // Extract yaw from quaternion
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

        RCLCPP_INFO(this->get_logger(), "Sentry planner node initialized (ctrl@%.0fHz, kp=%.1f)",
                     ctrl_freq, kp_);
    }

private:
    // ==================== Planning ====================
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!has_odom_) {
            RCLCPP_WARN(this->get_logger(), "No odom yet, cannot plan");
            return;
        }

        Eigen::Vector2d goal_pt(msg->pose.position.x, msg->pose.position.y);

        // Store final goal for re-planning
        final_goal_ = goal_pt;
        has_final_goal_ = true;

        planToGoal(goal_pt);
    }

    void planToGoal(Eigen::Vector2d goal_pt)
    {
        // Clamp goal to within the local ESDF range
        Eigen::Vector2d diff = goal_pt - current_pos_;
        double local_range = 7.5;
        this->get_parameter("sdf_map.local_update_range_x", local_range);
        local_range -= 0.5;
        bool goal_clamped = false;
        if (diff.norm() > local_range) {
            goal_pt = current_pos_ + diff.normalized() * local_range;
            goal_clamped = true;
        }

        // Clamp to map boundary
        Eigen::Vector2d map_origin, map_size;
        sdf_map_->getRegion(map_origin, map_size);
        Eigen::Vector2d map_max = map_origin + map_size;
        for (int i = 0; i < 2; ++i) {
            goal_pt(i) = std::max(goal_pt(i), map_origin(i) + 0.5);
            goal_pt(i) = std::min(goal_pt(i), map_max(i) - 0.5);
        }

        int start_occ = sdf_map_->getInflateOccupancy(current_pos_);
        double start_dist = sdf_map_->getDistance(current_pos_);
        RCLCPP_INFO(this->get_logger(),
            "Plan: (%.2f,%.2f)->(%.2f,%.2f)%s occ=%d dist=%.2f",
            current_pos_(0), current_pos_(1), goal_pt(0), goal_pt(1),
            goal_clamped ? " [clamped]" : "", start_occ, start_dist);

        // A* search
        kino_astar_->reset();
        int result = kino_astar_->search(
            current_pos_, current_vel_, Eigen::Vector2d::Zero(),
            goal_pt, Eigen::Vector2d::Zero(), true);

        if (result == KinodynamicAstar::NO_PATH) {
            RCLCPP_WARN(this->get_logger(), "No path found!");
            has_traj_ = false;
            return;
        }

        // B-spline parameterization
        double ts;
        std::vector<Eigen::Vector2d> point_set, start_end_derivatives;
        kino_astar_->getSamples(ts, point_set, start_end_derivatives);

        if (point_set.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Too few samples from A*");
            has_traj_ = false;
            return;
        }

        Eigen::MatrixXd ctrl_pts;
        NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

        // Optimize
        Eigen::MatrixXd opt_ctrl_pts = bspline_opt_->BsplineOptimizeTraj(
            ctrl_pts, ts, BsplineOptimizer::NORMAL_PHASE, 1, 1);

        // Build trajectory
        NonUniformBspline traj(opt_ctrl_pts, 3, ts);
        double max_vel = 3.0, max_acc = 3.0;
        this->get_parameter("search.max_vel", max_vel);
        this->get_parameter("search.max_acc", max_acc);
        traj.setPhysicalLimits(max_vel, max_acc);
        traj.reallocateTime();

        // Store for controller
        current_traj_ = traj;
        current_traj_vel_ = traj.getDerivative();
        traj_duration_ = traj.getTimeSum();
        traj_start_time_ = this->now();
        has_traj_ = true;

        // Publish visualization
        publishTrajectory(traj);

        RCLCPP_INFO(this->get_logger(), "Trajectory: %ld ctrl pts, duration=%.2fs",
                     opt_ctrl_pts.rows(), traj_duration_);
    }

    // ==================== Controller ====================
    void controlLoop()
    {
        if (!has_traj_ || !has_odom_)
            return;

        double elapsed = (this->now() - traj_start_time_).seconds();

        // Trajectory finished?
        if (elapsed >= traj_duration_) {
            // Stop the robot
            geometry_msgs::msg::Twist cmd;
            cmd_vel_pub_->publish(cmd);
            has_traj_ = false;

            // If we have a far-away final goal, re-plan toward it
            if (has_final_goal_) {
                double remaining = (final_goal_ - current_pos_).norm();
                if (remaining > 0.3) {
                    RCLCPP_INFO(this->get_logger(),
                        "Segment done, %.1fm to final goal, re-planning...", remaining);
                    planToGoal(final_goal_);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Reached final goal!");
                    has_final_goal_ = false;
                }
            }
            return;
        }

        // Sample desired position and velocity from trajectory
        Eigen::VectorXd pos_d = current_traj_.evaluateDeBoorT(elapsed);
        Eigen::VectorXd vel_d = current_traj_vel_.evaluateDeBoorT(elapsed);

        // PD control in odom frame: cmd = vel_ff + Kp * pos_error
        Eigen::Vector2d pos_err = pos_d.head(2) - current_pos_;
        Eigen::Vector2d vel_cmd_odom = vel_d.head(2) + kp_ * pos_err;

        // Clamp velocity
        double max_vel = 3.0;
        this->get_parameter("search.max_vel", max_vel);
        if (vel_cmd_odom.norm() > max_vel) {
            vel_cmd_odom = vel_cmd_odom.normalized() * max_vel;
        }

        // Transform odom-frame velocity to body frame (rotate by -yaw)
        double cy = cos(-current_yaw_), sy = sin(-current_yaw_);
        Eigen::Vector2d vel_body;
        vel_body(0) = cy * vel_cmd_odom(0) - sy * vel_cmd_odom(1);
        vel_body(1) = sy * vel_cmd_odom(0) + cy * vel_cmd_odom(1);

        // Publish cmd_vel
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = vel_body(0);
        cmd.linear.y = vel_body(1);
        cmd_vel_pub_->publish(cmd);
    }

    // ==================== Visualization ====================
    void publishTrajectory(NonUniformBspline &traj)
    {
        double t_start = 0.0;
        double t_end = traj.getTimeSum();

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "odom";

        visualization_msgs::msg::MarkerArray markers;

        // Delete old markers
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header = path_msg.header;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(delete_marker);

        // Trajectory line
        visualization_msgs::msg::Marker line_marker;
        line_marker.header = path_msg.header;
        line_marker.ns = "trajectory";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        line_marker.scale.x = 0.08;
        line_marker.color.r = 0.0;
        line_marker.color.g = 1.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 1.0;

        double dt = 0.05;
        for (double t = t_start; t <= t_end; t += dt) {
            Eigen::VectorXd pt = traj.evaluateDeBoorT(t);

            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = pt(0);
            pose.pose.position.y = pt(1);
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);

            geometry_msgs::msg::Point p;
            p.x = pt(0);
            p.y = pt(1);
            p.z = 0.0;
            line_marker.points.push_back(p);
        }

        markers.markers.push_back(line_marker);

        path_pub_->publish(path_msg);
        marker_pub_->publish(markers);
    }

    // ==================== Components ====================
    std::shared_ptr<SDFMap> sdf_map_;
    std::shared_ptr<EDTEnvironment> edt_env_;
    std::shared_ptr<KinodynamicAstar> kino_astar_;
    std::shared_ptr<BsplineOptimizer> bspline_opt_;

    // ==================== State ====================
    Eigen::Vector2d current_pos_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d current_vel_ = Eigen::Vector2d::Zero();
    double current_yaw_ = 0.0;
    bool has_odom_ = false;

    // Trajectory tracking
    NonUniformBspline current_traj_;
    NonUniformBspline current_traj_vel_;
    double traj_duration_ = 0.0;
    rclcpp::Time traj_start_time_;
    bool has_traj_ = false;

    // Final goal for re-planning
    Eigen::Vector2d final_goal_ = Eigen::Vector2d::Zero();
    bool has_final_goal_ = false;

    // Controller gains
    double kp_ = 2.0;

    // ==================== ROS interfaces ====================
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
