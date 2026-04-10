#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sentry_global_planner/global_map.hpp>
#include <sentry_global_planner/jps_searcher.hpp>

using namespace sentry_global;

class GlobalPlannerNode : public rclcpp::Node
{
public:
    GlobalPlannerNode() : Node("sentry_global_planner")
    {
        this->declare_parameter<std::string>("global_map.mode", "prior");
        this->declare_parameter<std::string>("global_map.yaml_path", "");
        this->declare_parameter<double>("jps.esdf_weight", 2.0);
        this->declare_parameter<double>("jps.safety_dist", 0.3);
        this->declare_parameter<double>("jps.waypoint_spacing", 1.0);
        this->declare_parameter<std::string>("manager.odometry", "/Odometry");
    }

    void initialize()
    {
        std::string mode = this->get_parameter("global_map.mode").as_string();

        if (mode == "prior")
        {
            auto prior = std::make_unique<PriorMap>();
            std::string yaml_path = this->get_parameter("global_map.yaml_path").as_string();
            RCLCPP_INFO(this->get_logger(), "Loading prior map: %s", yaml_path.c_str());
            prior->loadFromYaml(yaml_path);
            prior->computeESDF();

            // Debug: count occupied cells
            int occ_count = 0, total = prior->width() * prior->height();
            for (int x = 0; x < prior->width(); ++x)
                for (int y = 0; y < prior->height(); ++y)
                    if (prior->isOccupied(x, y)) occ_count++;
            RCLCPP_INFO(this->get_logger(),
                "Prior map ESDF computed: %dx%d, res=%.3f, occupied=%d/%d (%.1f%%)",
                prior->width(), prior->height(), prior->resolution(),
                occ_count, total, 100.0 * occ_count / total);
            prior_map_ = std::move(prior);
            map_ = prior_map_.get();
        }
        else
        {
/* PLACEHOLDER_ONLINE_INIT */
            auto sdf = std::make_shared<SDFMap>();
            sdf->initMap(this->shared_from_this());
            sdf_map_ = sdf;
            auto proxy = std::make_unique<OnlineMapProxy>();
            proxy->setSDFMap(sdf_map_);
            online_proxy_ = std::move(proxy);
            map_ = online_proxy_.get();
            RCLCPP_INFO(this->get_logger(), "Online mode: using SDFMap ESDF");
        }

        jps_.setMap(map_);
        jps_.setParams(
            this->get_parameter("jps.esdf_weight").as_double(),
            this->get_parameter("jps.safety_dist").as_double(),
            this->get_parameter("jps.waypoint_spacing").as_double());

        // Subscribers
        std::string odom_topic = this->get_parameter("manager.odometry").as_string();
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                current_pos_(0) = msg->pose.pose.position.x;
                current_pos_(1) = msg->pose.pose.position.y;
                has_odom_ = true;
            });

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&GlobalPlannerNode::goalCallback, this, std::placeholders::_1));

        // Publishers
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/global_waypoints", 10);
        esdf_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/global_esdf", 10);

        // Publish ESDF visualization periodically (prior mode)
        if (prior_map_)
        {
            vis_timer_ = this->create_wall_timer(
                std::chrono::seconds(5),
                [this]() { publishESDF(); });
        }

        RCLCPP_INFO(this->get_logger(), "Global planner initialized");
    }

private:
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!has_odom_) {
            RCLCPP_WARN(this->get_logger(), "No odom yet");
            return;
        }

        Eigen::Vector2d goal(msg->pose.position.x, msg->pose.position.y);
        RCLCPP_INFO(this->get_logger(), "Global plan: (%.2f,%.2f) -> (%.2f,%.2f)",
                    current_pos_(0), current_pos_(1), goal(0), goal(1));

        std::vector<Eigen::Vector2d> path;
        auto t0 = this->now();
        bool ok = jps_.search(current_pos_, goal, path);
        auto t1 = this->now();

        if (!ok) {
            RCLCPP_WARN(this->get_logger(), "Global JPS: no path found!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Global JPS: %zu waypoints, %.1fms",
                    path.size(), (t1 - t0).seconds() * 1000.0);

        // Publish path
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "odom";
        for (auto &pt : path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = pt(0);
            pose.pose.position.y = pt(1);
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        path_pub_->publish(path_msg);

        // Publish waypoint markers
        publishWaypoints(path);
    }

    void publishWaypoints(const std::vector<Eigen::Vector2d> &path)
    {
        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::Marker del;
        del.header.stamp = this->now();
        del.header.frame_id = "odom";
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(del);

        for (size_t i = 0; i < path.size(); ++i) {
            visualization_msgs::msg::Marker m;
            m.header.stamp = this->now();
            m.header.frame_id = "odom";
            m.ns = "global_wp";
            m.id = (int)i;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = path[i](0);
            m.pose.position.y = path[i](1);
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = m.scale.z = 0.3;
            m.color.r = 1.0; m.color.g = 0.5; m.color.a = 1.0;
            markers.markers.push_back(m);
        }
        marker_pub_->publish(markers);
    }

    void publishESDF()
    {
        if (!prior_map_) return;
        pcl::PointCloud<pcl::PointXYZI> cloud;
        int step = 1; // 全分辨率
        double max_vis_dist = 5.0;  // 可视化最大距离
        for (int x = 0; x < prior_map_->width(); x += step)
            for (int y = 0; y < prior_map_->height(); y += step) {
                double d = prior_map_->getESDF(x, y);
                // 跳过障碍内部 (d <= 0) 和 unknown/occupied 格子
                if (d <= 0 || prior_map_->isOccupied(x, y))
                    continue;
                auto pos = prior_map_->gridToWorld(x, y);
                pcl::PointXYZI pt;
                pt.x = pos(0); pt.y = pos(1); pt.z = -0.5;
                // intensity: 近障碍=1(红), 远障碍=0(蓝)
                pt.intensity = 1.0 - std::min(d / max_vis_dist, 1.0);
                cloud.push_back(pt);
            }
        cloud.width = cloud.size(); cloud.height = 1; cloud.is_dense = true;
        cloud.header.frame_id = "odom";
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.stamp = this->now();
        esdf_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published global ESDF: %zu pts", cloud.size());
    }

    // Map backends
    std::unique_ptr<PriorMap> prior_map_;
    std::unique_ptr<OnlineMapProxy> online_proxy_;
    SDFMap::Ptr sdf_map_;
    MapInterface *map_ = nullptr;

    JPSSearcher jps_;

    // State
    Eigen::Vector2d current_pos_ = Eigen::Vector2d::Zero();
    bool has_odom_ = false;

    // ROS
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_pub_;
    rclcpp::TimerBase::SharedPtr vis_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalPlannerNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
