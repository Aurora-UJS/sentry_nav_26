#ifndef _SDF_MAP_HPP_
#define _SDF_MAP_HPP_

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.hpp>

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <plan_env/map_core.hpp>
#include <plan_env/sensor_processor.hpp>

using namespace std;

class SDFMap
{
public:
  SDFMap() {};
  ~SDFMap() {};

  typedef std::shared_ptr<SDFMap> Ptr;

  void initMap(std::shared_ptr<rclcpp::Node> nh);

  // ==================== 委托到 MapCore ====================
  void posToIndex(const Eigen::Vector2d &pos, Eigen::Vector2i &id) { core_.posToIndex(pos, id); }
  void indexToPos(const Eigen::Vector2i &id, Eigen::Vector2d &pos) { core_.indexToPos(id, pos); }
  bool isInMap(const Eigen::Vector2d &pos) { return core_.isInMap(pos); }
  bool isInMap(const Eigen::Vector2i &idx) { return core_.isInMap(idx); }
  void boundIndex(Eigen::Vector2i &id) { core_.boundIndex(id); }
  int toAddress(const Eigen::Vector2i &id) { return core_.toAddress(id); }
  int toAddress(int &x, int &y) { return core_.toAddress(x, y); }
  double getDistance(const Eigen::Vector2d &pos) { return core_.getDistance(pos); }
  double getDistanceByIndex(const Eigen::Vector2i &idx) { return core_.getDistanceByIndex(idx); }
  int getInflateOccupancy(Eigen::Vector2d pos) { return core_.getInflateOccupancy(pos); }
  double getResolution() { return core_.getResolution(); }
  void getRegion(Eigen::Vector2d &ori, Eigen::Vector2d &size) { core_.getRegion(ori, size); }
  Eigen::Vector2d getMapOrigin() const { return core_.getMapOrigin(); }
  void getSurroundPts(const Eigen::Vector2d &pos, Eigen::Vector2d pts[2][2], Eigen::Vector2d &diff)
  { core_.getSurroundPts(pos, pts, diff); }
  void setLocalMap(int num) { core_.setLocalMap(num); }

  // 直接暴露 core 供 edt_environment 使用
  MapCore& getCore() { return core_; }

  Global_Map load_map(std::string &path, const std::string &frame, int &map_buffer_size, MappingParameters &mp);

private:
  MapCore core_;
  SensorProcessor sensor_proc_;

  std::shared_ptr<rclcpp::Node> node_;

  // ROS subscribers/publishers/timers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  bool use_cloud_input_ = false;

  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> tf2_filter_;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> laser_sub_;
  laser_geometry::LaserProjection projectoir_;

  rclcpp::TimerBase::SharedPtr esdf_timer_, vis_timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_, esdf_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  std::vector<std::shared_ptr<nav_msgs::msg::OccupancyGrid>> map_msgs;

  // 高度过滤参数 (ROS 参数读取后缓存)
  double cloud_min_height_ = -0.1, cloud_max_height_ = 1.0;

  // ROS 回调
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom);
  void laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser_msg);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg);

  void updateESDFCallback();
  void visCallback();
  void publishMap();
  void publishESDF();
  void publish_map();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#endif
