#include "plan_env/sdf_map.hpp"

using namespace std;

void SDFMap::initMap(std::shared_ptr<rclcpp::Node> nh)
{
	this->node_ = nh;
	map_publisher_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("map_topic", 10);

	auto &mp = core_.mp_;
	auto &md = core_.md_;

	mp.frame_id_ = node_->declare_parameter<std::string>("sdf_map.frame_id", std::string("odom"));

	md.use_global_map = node_->declare_parameter<bool>("sdf_map.use_global_map", true);
	md.use_global_map = node_->get_parameter("sdf_map.use_global_map").as_bool();
	md.global_map_num = node_->declare_parameter<int>("sdf_map.global_map_num", 2);
	md.global_map_num = node_->get_parameter("sdf_map.global_map_num").as_int();
	node_->declare_parameter<std::vector<std::string>>("sdf_map.global_map_path", std::vector<std::string>());
	std::vector<std::string> global_map_path;
	node_->get_parameter("sdf_map.global_map_path", global_map_path);

	if (md.use_global_map)
	{
		if (static_cast<int>(global_map_path.size()) < md.global_map_num)
		{
			RCLCPP_ERROR(node_->get_logger(), "Invalid number of elements in 'global_map_path': expected %d, got %zu",
						 md.global_map_num, global_map_path.size());
			throw std::runtime_error("Parameter 'global_map_path' has invalid size.");
		}
		int max_map_buffer_size = std::numeric_limits<int>::min();
		for (int i = 0; i < md.global_map_num; i++)
		{
			int map_buffer_size = 0;
			md.Global_Maps.push_back(load_map(global_map_path[i], mp.frame_id_, map_buffer_size, mp));

			if (max_map_buffer_size < map_buffer_size)
				max_map_buffer_size = map_buffer_size;
		}
		core_.initBuffers(mp, max_map_buffer_size);
	}
	else
	{
		mp.map_size_ = {node_->declare_parameter<double>("sdf_map.map_size_x", 30.0),
						 node_->declare_parameter<double>("sdf_map.map_size_y", 30.0)};

		mp.resolution_ = node_->declare_parameter<double>("sdf_map.resolusion_", 0.01);
		mp.map_origin_ = {node_->declare_parameter<double>("sdf_map.origin_x", -6.35),
						   node_->declare_parameter<double>("sdf_map.origin_y", -7.6)};
		mp.map_origin_ = Eigen::Vector2d(-mp.map_size_(0) / 2.0, -mp.map_size_(1) / 2.0);
		for (int i = 0; i < 2; ++i)
			mp.map_voxel_num_(i) = ceil(mp.map_size_(i) / mp.resolution_);

		RCLCPP_INFO(node_->get_logger(),
			"SDF Map: resolution=%.3f, voxels=%dx%d, origin=(%.1f,%.1f), size=(%.1f,%.1f)",
			mp.resolution_, mp.map_voxel_num_(0), mp.map_voxel_num_(1),
			mp.map_origin_(0), mp.map_origin_(1), mp.map_size_(0), mp.map_size_(1));

		int buffer_size = mp.map_voxel_num_(0) * mp.map_voxel_num_(1);
		core_.initBuffers(mp, buffer_size);
	}

	esdf_timer_ = node_->create_wall_timer(0.05s, std::bind(&SDFMap::updateESDFCallback, this));
	mp.obstacles_inflation_ = node_->declare_parameter<double>("sdf_map.obstacles_inflation", 0.0009);
	mp.max_slope_rad_ = node_->declare_parameter<double>("sdf_map.max_slope_deg", 17.0) * M_PI / 180.0;
	mp.cloud_min_h_ = node_->declare_parameter<double>("sdf_map.cloud_min_height", -0.2);
	mp.cloud_max_h_ = node_->declare_parameter<double>("sdf_map.cloud_max_height", 0.15);
	mp.normal_voxel_leaf_ = node_->declare_parameter<double>("sdf_map.normal_voxel_leaf", 0.08);
	mp.normal_k_ = node_->declare_parameter<int>("sdf_map.normal_k", 10);
	mp.normal_count_thresh_ = node_->declare_parameter<int>("sdf_map.normal_count_thresh", 3);
	mp.local_map_margin_ = node_->declare_parameter<int>("sdf_map.local_map_margin", 10);
	mp.local_update_range_(0) = node_->declare_parameter<double>("sdf_map.local_update_range_x", 3.0);
	mp.local_update_range_(1) = node_->declare_parameter<double>("sdf_map.local_update_range_y", 3.0);
	mp.resolution_inv_ = 1 / mp.resolution_;

	// Log-odds 参数
	mp.logodds_hit_ = node_->declare_parameter<double>("sdf_map.logodds_hit", 0.85);
	mp.logodds_miss_ = node_->declare_parameter<double>("sdf_map.logodds_miss", -0.4);
	mp.logodds_max_ = node_->declare_parameter<double>("sdf_map.logodds_max", 3.5);
	mp.logodds_min_ = node_->declare_parameter<double>("sdf_map.logodds_min", -2.0);
	mp.logodds_thresh_ = node_->declare_parameter<double>("sdf_map.logodds_thresh", 0.0);
	mp.occ_timeout_ = node_->declare_parameter<double>("sdf_map.occ_timeout", 0.0);

	RCLCPP_INFO(node_->get_logger(),
		"Inflation: %.3fm (inf_step=%d), local_range=(%.1f,%.1f)",
		mp.obstacles_inflation_, (int)ceil(mp.obstacles_inflation_ / mp.resolution_),
		mp.local_update_range_(0), mp.local_update_range_(1));
	RCLCPP_INFO(node_->get_logger(),
		"SDF Init: origin=(%.2f,%.2f), res=%.4f, res_inv=%.2f, voxels=(%d,%d)",
		mp.map_origin_(0), mp.map_origin_(1), mp.resolution_, mp.resolution_inv_,
		mp.map_voxel_num_(0), mp.map_voxel_num_(1));

	mp.show_esdf_time_ = node_->declare_parameter<bool>("sdf_map.show_esdf_time", false);

	mp.map_min_boundary_ = mp.map_origin_;
	mp.map_max_boundary_ = mp.map_origin_ + mp.map_size_;

	// 初始化 sensor processor
	sensor_proc_.setMapCore(&core_);

	vis_timer_ = node_->create_wall_timer(0.05s, std::bind(&SDFMap::visCallback, this));
	map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/occupancy", 10);
	esdf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/esdf", 10);

	std::string odom_topic = node_->get_parameter("manager.odometry").as_string();
	RCLCPP_INFO(node_->get_logger(), "Odometry topic: %s", odom_topic.c_str());
	odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10,
		std::bind(&SDFMap::odomCallback, this, std::placeholders::_1));

	use_cloud_input_ = node_->declare_parameter<bool>("sdf_map.use_cloud_input", false);
	if (use_cloud_input_)
	{
		std::string cloud_topic = node_->declare_parameter<std::string>("sdf_map.cloud_topic", "/cloud_registered");
		RCLCPP_INFO(node_->get_logger(), "Using PointCloud2 input on topic: %s", cloud_topic.c_str());

		cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
			cloud_topic, rclcpp::SensorDataQoS(),
			std::bind(&SDFMap::cloudCallback, this, std::placeholders::_1));
	}
	else
	{
		std::string laser_topic = node_->declare_parameter<std::string>("sdf_map.laser_topic", "scan");
		laser_sub_.subscribe(node_, laser_topic, rmw_qos_profile_sensor_data);

		tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
		auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
			node_->get_node_base_interface(), node_->get_node_timers_interface());
		tf2_buffer_->setCreateTimerInterface(timer_interface);
		tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
		tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
			laser_sub_, *tf2_buffer_, "odom", 10, node_->get_node_logging_interface(),
			node_->get_node_clock_interface(), std::chrono::duration<int>(1));
		tf2_filter_->registerCallback(&SDFMap::laserCallback, this);
	}
}

Global_Map SDFMap::load_map(std::string &path, const std::string &frame, int &map_buffer_size, MappingParameters &mp)
{
	Global_Map gm;
	gm.path = path;
	cv::FileStorage fs(path, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		RCLCPP_ERROR(rclcpp::get_logger("load_map"), "Failed to open YAML file");
		throw std::runtime_error("Failed to open YAML file.");
	}

	float resolution = 0.0;
	float occupied_thresh = 0.0;
	cv::Point3f origin;
	fs["resolution"] >> resolution;
	fs["origin"] >> origin;
	fs["occupied_thresh"] >> occupied_thresh;

	if (resolution <= 0.0)
	{
		RCLCPP_ERROR(rclcpp::get_logger("load_map"), "Invalid map resolution: %f", resolution);
		throw std::invalid_argument("Invalid map resolution.");
	}
	gm.resolution_ = resolution;
	gm.map_origin_ = {origin.x, origin.y};

	size_t dot_pos = path.find_last_of('.');
	if (dot_pos != std::string::npos)
		path.replace(dot_pos, path.length() - dot_pos, ".pgm");

	cv::Mat image = cv::imread(path, cv::IMREAD_GRAYSCALE);
	if (image.empty())
	{
		RCLCPP_ERROR(rclcpp::get_logger("load_map"), "Failed to load PGM image");
		throw std::runtime_error("Failed to load PGM image.");
	}

	gm.image_size = {image.rows, image.cols};
	gm.map_size_ = {image.cols * gm.resolution_, image.rows * gm.resolution_};

	for (int i = 0; i < 2; ++i)
		gm.map_voxel_num_(i) = ceil(gm.map_size_(i) / gm.resolution_);

	int buffer_size = gm.map_voxel_num_(0) * gm.map_voxel_num_(1);
	gm.occupancy_buffer_inflate_Global_Map = vector<char>(buffer_size, 0);

	cv::normalize(image, image, 0, 1, cv::NORM_MINMAX);

	auto msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
	msg->header.stamp = rclcpp::Clock().now();
	msg->header.frame_id = frame;
	msg->info.resolution = gm.resolution_;
	msg->info.width = image.cols;
	msg->info.height = image.rows;
	msg->info.origin.position.x = gm.map_origin_[0];
	msg->info.origin.position.y = gm.map_origin_[1];
	msg->info.origin.position.z = 0.0;
	msg->data.resize(gm.map_voxel_num_(0) * gm.map_voxel_num_(1));
	for (int y = 0; y < image.rows; ++y)
		for (int x = 0; x < image.cols; ++x)
		{
			int pixelValue = image.at<uchar>(y, x);
			msg->data[image.cols * (image.rows - y - 1) + x] = (pixelValue < occupied_thresh) ? 100 : 0;
		}
	map_msgs.push_back(msg);
	cv::flip(image, image, 0);

	for (int y = 0; y < image.rows; ++y)
		for (int x = 0; x < image.cols; ++x)
		{
			int pixelValue = image.at<uchar>(y, x);
			if (pixelValue < occupied_thresh)
				gm.occupancy_buffer_inflate_Global_Map[x * gm.map_voxel_num_(1) + y] = 1;
		}

	mp.resolution_ = gm.resolution_;
	mp.map_origin_ = gm.map_origin_;
	mp.map_size_(0) = std::max(image.cols * gm.resolution_, mp.map_size_(0));
	mp.map_size_(1) = std::max(image.rows * gm.resolution_, mp.map_size_(1));
	mp.map_voxel_num_(0) = std::max(gm.map_voxel_num_(0), mp.map_voxel_num_(0));
	mp.map_voxel_num_(1) = std::max(gm.map_voxel_num_(1), mp.map_voxel_num_(1));
	map_buffer_size = std::max(map_buffer_size, buffer_size);
	return gm;
}

// ==================== ROS 回调 (薄壳) ====================

void SDFMap::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom)
{
	auto &md = core_.md_;
	md.laser_pos_(0) = odom->pose.pose.position.x;
	md.laser_pos_(1) = odom->pose.pose.position.y;
	md.laser_z_ = std::clamp(odom->pose.pose.position.z, -0.5, 0.5);
	md.laser_q_ = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
									  odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
	md.has_odom_ = true;
}

void SDFMap::laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser_msg)
{
	if (!core_.md_.has_odom_)
		return;

	auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
	projectoir_.projectLaser(*laser_msg, *cloud_msg);
	try
	{
		*cloud_msg = tf2_buffer_->transform(*cloud_msg, "odom", tf2::durationFromSec(0.1));
	}
	catch (const tf2::ExtrapolationException &ex)
	{
		RCLCPP_ERROR(node_->get_logger(), "Error while transforming %s", ex.what());
		return;
	}

	pcl::PointCloud<pcl::PointXY> latest_laser;
	pcl::fromROSMsg(*cloud_msg, latest_laser);
	core_.md_.has_cloud_ = true;
	if (latest_laser.points.empty())
		return;
	if (isnan(core_.md_.laser_pos_(0)) || isnan(core_.md_.laser_pos_(1)))
		return;

	sensor_proc_.processLaser(latest_laser, core_.md_.laser_pos_,
							  rclcpp::Time(laser_msg->header.stamp).seconds());
}

void SDFMap::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg)
{
	if (!core_.md_.has_odom_)
		return;

	pcl::PointCloud<pcl::PointXYZ> cloud_3d;
	pcl::fromROSMsg(*cloud_msg, cloud_3d);
	core_.md_.has_cloud_ = true;

	if (cloud_3d.points.empty())
		return;
	if (isnan(core_.md_.laser_pos_(0)) || isnan(core_.md_.laser_pos_(1)))
		return;

	RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
		"cloudCallback: %zu pts, robot at (%.2f, %.2f)",
		cloud_3d.points.size(), core_.md_.laser_pos_(0), core_.md_.laser_pos_(1));

	sensor_proc_.processCloud(cloud_3d, core_.md_.laser_pos_, core_.md_.laser_z_,
							  rclcpp::Time(cloud_msg->header.stamp).seconds());
}

void SDFMap::updateESDFCallback()
{
	if (!core_.md_.esdf_need_update_)
		return;

	RCLCPP_INFO_ONCE(node_->get_logger(), "ESDF update starting...");
	auto t1 = rclcpp::Clock().now();
	core_.updateESDF2d();
	auto t2 = rclcpp::Clock().now();
	RCLCPP_INFO_ONCE(node_->get_logger(), "ESDF update done in %.3fs", (t2 - t1).seconds());

	auto &md = core_.md_;
	md.esdf_time_ += (t2 - t1).seconds();
	md.max_esdf_time_ = max(md.max_esdf_time_, (t2 - t1).seconds());

	if (core_.mp_.show_esdf_time_)
		RCLCPP_INFO(node_->get_logger(), "ESDF: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).seconds(),
					md.esdf_time_ / md.update_num_, md.max_esdf_time_);

	md.esdf_need_update_ = false;
}

void SDFMap::publishMap()
{
	pcl::PointXYZ pt;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	auto &md = core_.md_;
	auto &mp = core_.mp_;

	Eigen::Vector2i min_cut = md.local_bound_min_;
	Eigen::Vector2i max_cut = md.local_bound_max_;

	int lmm = mp.local_map_margin_ / 2;
	min_cut -= Eigen::Vector2i(lmm, lmm);
	max_cut += Eigen::Vector2i(lmm, lmm);

	core_.boundIndex(min_cut);
	core_.boundIndex(max_cut);

	for (int x = min_cut(0); x <= max_cut(0); ++x)
		for (int y = min_cut(1); y <= max_cut(1); ++y)
		{
			if (md.is_occupancy(core_.toAddress(x, y)) == 0)
				continue;
			Eigen::Vector2d pos;
			core_.indexToPos(Eigen::Vector2i(x, y), pos);
			pt.x = pos(0); pt.y = pos(1); pt.z = 0;
			cloud.push_back(pt);
		}

	cloud.width = cloud.points.size();
	cloud.height = 1;
	cloud.is_dense = true;
	cloud.header.frame_id = mp.frame_id_;
	sensor_msgs::msg::PointCloud2 cloud_msg;
	pcl::toROSMsg(cloud, cloud_msg);
	cloud_msg.header.stamp = node_->now();
	map_pub_->publish(cloud_msg);
}

void SDFMap::publishESDF()
{
	double dist;
	pcl::PointCloud<pcl::PointXYZI> cloud;
	pcl::PointXYZI pt;
	auto &md = core_.md_;
	auto &mp = core_.mp_;

	const double min_dist = 0.0;
	const double max_dist = 3.0;

	Eigen::Vector2i min_cut = md.local_bound_min_;
	Eigen::Vector2i max_cut = md.local_bound_max_;
	core_.boundIndex(min_cut);
	core_.boundIndex(max_cut);

	for (int x = min_cut(0); x <= max_cut(0); ++x)
		for (int y = min_cut(1); y <= max_cut(1); ++y)
		{
			Eigen::Vector2d pos;
			core_.indexToPos(Eigen::Vector2i(x, y), pos);

			dist = core_.getDistance(pos);
			dist = min(dist, max_dist);
			dist = max(dist, min_dist);

			pt.x = pos(0); pt.y = pos(1); pt.z = 0;
			pt.intensity = 1.0 - (dist - min_dist) / (max_dist - min_dist);
			cloud.push_back(pt);
		}

	cloud.width = cloud.points.size();
	cloud.height = 1;
	cloud.is_dense = true;
	cloud.header.frame_id = mp.frame_id_;
	sensor_msgs::msg::PointCloud2 cloud_msg;
	pcl::toROSMsg(cloud, cloud_msg);
	cloud_msg.header.stamp = node_->now();
	esdf_pub_->publish(cloud_msg);
}

void SDFMap::publish_map()
{
	auto &md = core_.md_;
	if (!md.use_global_map || md.current_global_map > md.global_map_num || md.current_global_map < 0)
		return;
	map_publisher_->publish(*(map_msgs[md.current_global_map]));
}

void SDFMap::visCallback()
{
	if (!core_.md_.has_cloud_)
		return;
	publishMap();
	publish_map();
	publishESDF();
}
