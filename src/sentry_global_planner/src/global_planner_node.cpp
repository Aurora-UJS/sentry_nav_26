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
#include <plan_env/traversability_layer.hpp>

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
        this->declare_parameter<double>("jps.oneway_penalty", 50.0);
        this->declare_parameter<std::string>("manager.odometry", "/Odometry");
        this->declare_parameter<std::string>("traversability.yaml_path", "");
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
            this->get_parameter("jps.waypoint_spacing").as_double(),
            this->get_parameter("jps.oneway_penalty").as_double());

        // 可通行性标注层 (可选, 特性 OPT-IN): yaml_path 为空 / 加载失败 → 不挂载, 行为与原先完全一致
        std::string trav_path = this->get_parameter("traversability.yaml_path").as_string();
        if (!trav_path.empty())
        {
            auto layer = std::make_shared<sentry_nav::TraversabilityLayer>();
            if (layer->loadFromYaml(trav_path))
            {
                trav_layer_ = layer;
                jps_.setTravLayer(trav_layer_.get());
                RCLCPP_INFO(this->get_logger(),
                    "Traversability layer loaded: %s (%zu regions)",
                    trav_path.c_str(), trav_layer_->regions().size());
            }
            else
            {
                // 空 regions / 解析失败均返回 false → 层禁用 (查询全部按 FREE/放行), 行为同未启用
                RCLCPP_INFO(this->get_logger(),
                    "Traversability layer disabled (empty or unparsable): %s", trav_path.c_str());
            }
        }

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
        trav_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/traversability_layer", 10);

        // Publish ESDF visualization periodically (prior mode)
        if (prior_map_)
        {
            vis_timer_ = this->create_wall_timer(
                std::chrono::seconds(5),
                [this]() { publishESDF(); });
        }

        // 可通行性标注层可视化 (周期发布, 同 ESDF 5s 定时器约定)
        if (trav_layer_)
        {
            trav_timer_ = this->create_wall_timer(
                std::chrono::seconds(5),
                [this]() { publishTravLayer(); });
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

    // 发布可通行性标注层可视化: 每个区域画闭合多边形轮廓 (按类型着色),
    // ONEWAY 区在质心额外画一支沿 direction_deg 的箭头。
    void publishTravLayer()
    {
        if (!trav_layer_ || !trav_layer_->enabled()) return;
        const auto &regions = trav_layer_->regions();

        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::Marker del;
        del.header.stamp = this->now();
        del.header.frame_id = "odom";
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(del);

        int mid = 0;
        for (const auto &reg : regions)
        {
            if (reg.polygon.size() < 2) continue;

            // 区域边界 (闭合 LINE_STRIP), 按类型着色: obstacle=红, oneway=橙, free=绿
            visualization_msgs::msg::Marker line;
            line.header.stamp = this->now();
            line.header.frame_id = "odom";
            line.ns = "trav_region";
            line.id = mid++;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.pose.orientation.w = 1.0;
            line.scale.x = 0.08;  // 线宽
            line.color.a = 1.0;
            switch (reg.type)
            {
                case sentry_nav::TravType::OBSTACLE:  // 红
                    line.color.r = 1.0; line.color.g = 0.0; line.color.b = 0.0; break;
                case sentry_nav::TravType::ONEWAY:    // 橙
                    line.color.r = 1.0; line.color.g = 0.55; line.color.b = 0.0; break;
                default:                              // 绿 (free)
                    line.color.r = 0.0; line.color.g = 1.0; line.color.b = 0.0; break;
            }
            Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
            for (const auto &v : reg.polygon)
            {
                geometry_msgs::msg::Point p;
                p.x = v(0); p.y = v(1); p.z = 0.0;
                line.points.push_back(p);
                centroid += v;
            }
            // 闭合: 回到首顶点
            geometry_msgs::msg::Point p0;
            p0.x = reg.polygon.front()(0); p0.y = reg.polygon.front()(1); p0.z = 0.0;
            line.points.push_back(p0);
            centroid /= (double)reg.polygon.size();
            markers.markers.push_back(line);

            // ONEWAY: 质心处沿 direction_deg 画箭头
            if (reg.type == sentry_nav::TravType::ONEWAY)
            {
                visualization_msgs::msg::Marker arrow;
                arrow.header.stamp = this->now();
                arrow.header.frame_id = "odom";
                arrow.ns = "trav_arrow";
                arrow.id = mid++;
                arrow.type = visualization_msgs::msg::Marker::ARROW;
                arrow.action = visualization_msgs::msg::Marker::ADD;
                arrow.pose.orientation.w = 1.0;
                const double deg2rad = 0.017453292519943295;
                double rad = reg.dir_deg * deg2rad;
                double len = 0.8;  // 箭头长度 (m)
                geometry_msgs::msg::Point tail, tip;
                tail.x = centroid(0); tail.y = centroid(1); tail.z = 0.0;
                tip.x = centroid(0) + len * std::cos(rad);
                tip.y = centroid(1) + len * std::sin(rad);
                tip.z = 0.0;
                arrow.points.push_back(tail);
                arrow.points.push_back(tip);
                arrow.scale.x = 0.08;  // 杆径
                arrow.scale.y = 0.18;  // 箭头径
                arrow.scale.z = 0.0;
                arrow.color.a = 1.0;
                arrow.color.r = 1.0; arrow.color.g = 0.55; arrow.color.b = 0.0;
                markers.markers.push_back(arrow);
            }
        }
        trav_pub_->publish(markers);
        RCLCPP_INFO(this->get_logger(), "Published traversability layer: %zu regions",
                    regions.size());
    }

    // Map backends
    std::unique_ptr<PriorMap> prior_map_;
    std::unique_ptr<OnlineMapProxy> online_proxy_;
    SDFMap::Ptr sdf_map_;
    MapInterface *map_ = nullptr;
    std::shared_ptr<sentry_nav::TraversabilityLayer> trav_layer_;  // 可选标注层 (持有生命周期)

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
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trav_pub_;
    rclcpp::TimerBase::SharedPtr vis_timer_;
    rclcpp::TimerBase::SharedPtr trav_timer_;
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
