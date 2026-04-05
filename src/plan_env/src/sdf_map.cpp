#include "plan_env/sdf_map.hpp"

void SDFMap::initMap(std::shared_ptr<rclcpp::Node> nh)
{
	this->node_ = nh;
	map_publisher_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("map_topic", 10);
	mp_.frame_id_ = node_->declare_parameter<std::string>("sdf_map.frame_id", std::string("odom"));

	md_.use_global_map = node_->declare_parameter<bool>("sdf_map.use_global_map", true);
	md_.use_global_map = node_->get_parameter("sdf_map.use_global_map").as_bool();
	md_.global_map_num = node_->declare_parameter<int>("sdf_map.global_map_num", 2);
	md_.global_map_num = node_->get_parameter("sdf_map.global_map_num").as_int();
	node_->declare_parameter<std::vector<std::string>>("sdf_map.global_map_path", std::vector<std::string>());
	std::vector<std::string> global_map_path;
	node_->get_parameter("sdf_map.global_map_path", global_map_path);
	if (md_.use_global_map)
	{
		if (static_cast<int>(global_map_path.size()) < md_.global_map_num)
		{
			RCLCPP_ERROR(node_->get_logger(), "Invalid number of elements in 'global_map_path': expected %d, got %zu",
						 md_.global_map_num, global_map_path.size());
			throw std::runtime_error("Parameter 'global_map_path' has invalid size.");
		}
		int max_map_buffer_size = std::numeric_limits<int>::min();
		for (int i = 0; i < md_.global_map_num; i++)
		{
			int map_buffer_size = 0;
			md_.Global_Maps.push_back(load_map(global_map_path[i], mp_.frame_id_, map_buffer_size, mp_));

			if (max_map_buffer_size < map_buffer_size)
				max_map_buffer_size = map_buffer_size;
		}
		md_.occupancy_buffer_neg = vector<char>(max_map_buffer_size, 0);
		md_.occupancy_buffer_inflate_ = vector<char>(max_map_buffer_size, 0);
		md_.distance_buffer_ = vector<double>(max_map_buffer_size, 10000);
		md_.distance_buffer_neg_ = vector<double>(max_map_buffer_size, 10000);
		md_.distance_buffer_all_ = vector<double>(max_map_buffer_size, 10000);
		md_.tmp_buffer1_ = vector<double>(max_map_buffer_size, 0);
		md_.elevation_buffer_ = vector<float>(max_map_buffer_size, std::numeric_limits<float>::quiet_NaN());
		md_.slope_obstacle_buffer_ = vector<char>(max_map_buffer_size, 0);
	}
	else
	{
		mp_.map_size_ = {node_->declare_parameter<double>("sdf_map.map_size_x", 30.0),
						 node_->declare_parameter<double>("sdf_map.map_size_y", 30.0)};

		mp_.resolution_ = node_->declare_parameter<double>("sdf_map.resolusion_", 0.01);
		mp_.map_origin_ = {node_->declare_parameter<double>("sdf_map.origin_x", -6.35),
						   node_->declare_parameter<double>("sdf_map.origin_y", -7.6)};
		mp_.map_origin_ = Eigen::Vector2d(-mp_.map_size_(0) / 2.0, -mp_.map_size_(1) / 2.0);
		for (int i = 0; i < 2; ++i)
			mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

		RCLCPP_INFO(node_->get_logger(),
			"SDF Map: resolution=%.3f, voxels=%dx%d, origin=(%.1f,%.1f), size=(%.1f,%.1f)",
			mp_.resolution_, mp_.map_voxel_num_(0), mp_.map_voxel_num_(1),
			mp_.map_origin_(0), mp_.map_origin_(1), mp_.map_size_(0), mp_.map_size_(1));

		int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1);

		md_.occupancy_buffer_neg = vector<char>(buffer_size, 0);
		md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
		md_.distance_buffer_ = vector<double>(buffer_size, 10000);
		md_.distance_buffer_neg_ = vector<double>(buffer_size, 10000);
		md_.distance_buffer_all_ = vector<double>(buffer_size, 10000);
		md_.tmp_buffer1_ = vector<double>(buffer_size, 0);
		md_.elevation_buffer_ = vector<float>(buffer_size, std::numeric_limits<float>::quiet_NaN());
		md_.slope_obstacle_buffer_ = vector<char>(buffer_size, 0);
	}

	esdf_timer_ = node_->create_wall_timer(0.05s, std::bind(&SDFMap::updateESDFCallback, this));
	mp_.obstacles_inflation_ = node_->declare_parameter<double>("sdf_map.obstacles_inflation", 0.0009);
	mp_.max_slope_rad_ = node_->declare_parameter<double>("sdf_map.max_slope_deg", 17.0) * M_PI / 180.0;
	mp_.step_height_max_ = node_->declare_parameter<double>("sdf_map.step_height_max", 0.08);
	mp_.local_map_margin_ = node_->declare_parameter<int>("sdf_map.local_map_margin", 10);
	mp_.local_update_range_(0) = node_->declare_parameter<double>("sdf_map.local_update_range_x", 3.0);
	mp_.local_update_range_(1) = node_->declare_parameter<double>("sdf_map.local_update_range_y", 3.0);
	mp_.resolution_inv_ = 1 / mp_.resolution_;

	RCLCPP_INFO(node_->get_logger(),
		"Inflation: %.3fm (inf_step=%d), local_range=(%.1f,%.1f)",
		mp_.obstacles_inflation_, (int)ceil(mp_.obstacles_inflation_ / mp_.resolution_),
		mp_.local_update_range_(0), mp_.local_update_range_(1));
	RCLCPP_INFO(node_->get_logger(),
		"SDF Init: origin=(%.2f,%.2f), res=%.4f, res_inv=%.2f, voxels=(%d,%d)",
		mp_.map_origin_(0), mp_.map_origin_(1), mp_.resolution_, mp_.resolution_inv_,
		mp_.map_voxel_num_(0), mp_.map_voxel_num_(1));
	// mp_.map_origin_ = Eigen::Vector2d(-x_size / 2.0, -y_size / 2.0);
	// mp_.map_size_ = Eigen::Vector2d(x_size, y_size);
	mp_.show_esdf_time_ = node_->declare_parameter<bool>("sdf_map.show_esdf_time", false);

	// md_.tmp_buffer1_ = vector<double>(buffer_size, 0);
	mp_.map_min_boundary_ = mp_.map_origin_;
	mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;
	md_.max_esdf_time_ = 0.0;

	// 初始化 local bounds 到地图中心附近 (防止未初始化的垃圾值)
	md_.local_bound_min_ = Eigen::Vector2i(0, 0);
	md_.local_bound_max_ = Eigen::Vector2i(0, 0);
	md_.local_bound_min_ = Eigen::Vector2i::Zero();
	md_.local_bound_max_ = Eigen::Vector2i::Zero();
	md_.has_odom_ = false;
	md_.has_cloud_ = false;
	md_.esdf_need_update_ = false;
	md_.update_num_ = 0;

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

		// Height filter parameters for projecting 3D -> 2D
		node_->declare_parameter<double>("sdf_map.cloud_min_height", -0.1);
		node_->declare_parameter<double>("sdf_map.cloud_max_height", 1.0);

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
Global_Map SDFMap::load_map(std::string &path, const std::string &frame,int& map_buffer_size, MappingParameters &mp)
{
	// 打开并读取YAML元数据
	Global_Map gm;
	gm.path = path;
	cv::FileStorage fs(path, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		RCLCPP_ERROR(rclcpp::get_logger("load_map"), "Failed to open YAML file");
		throw std::runtime_error("Failed to open YAML file.");
	}

	// 获取元数据中的map分辨率和原点信息
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
	{
		path.replace(dot_pos, path.length() - dot_pos, ".pgm");
	}
	else
	{
		std::cerr << "No file extension found in the input path!" << std::endl;
	}
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
	{
		for (int x = 0; x < image.cols; ++x)
		{
			int pixelValue = image.at<uchar>(y, x);
			if (pixelValue < occupied_thresh)
			{
				msg->data[image.cols * (image.rows - y - 1) + x] = 100;
			}
			else
				msg->data[image.cols * (image.rows - y - 1) + x] = 0;
		}
	}
	map_msgs.push_back(msg);
	cv::flip(image, image, 0);
	std::cout << "image.rows:" << image.rows << std::endl
			  << "image.cols" << image.cols << std::endl;

	for (int y = 0; y < image.rows; ++y)
	{
		for (int x = 0; x < image.cols; ++x)
		{
			int pixelValue = image.at<uchar>(y, x);
			if (pixelValue < occupied_thresh)
			{
				gm.occupancy_buffer_inflate_Global_Map[x * gm.map_voxel_num_(1) + y] = 1;
			}
		}
	}
	mp.resolution_ = gm.resolution_;
	mp.map_origin_ = gm.map_origin_;
	mp_.map_size_(0) = std::max(image.cols * gm.resolution_, mp_.map_size_(0));
	mp_.map_size_(1) = std::max(image.rows * gm.resolution_, mp_.map_size_(1));
	mp_.map_voxel_num_(0) = std::max(gm.map_voxel_num_(0), mp_.map_voxel_num_(0));
	mp_.map_voxel_num_(1) = std::max(gm.map_voxel_num_(1), mp_.map_voxel_num_(1));
	map_buffer_size = std::max(map_buffer_size,buffer_size);
	return gm;
}

void SDFMap::resetBuffer(Eigen::Vector2d min_pos, Eigen::Vector2d max_pos)
{

	Eigen::Vector2i min_id, max_id;
	posToIndex(min_pos, min_id);
	posToIndex(max_pos, max_id);

	boundIndex(min_id);
	boundIndex(max_id);

	/* reset occ and dist buffer */
	for (int x = min_id(0); x <= max_id(0); ++x)
		for (int y = min_id(1); y <= max_id(1); ++y)
		{
			md_.occupancy_buffer_inflate_[toAddress(x, y)] = 0;
			md_.distance_buffer_[toAddress(x, y)] = 10000;
		}
}
void SDFMap::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom)
{
	md_.laser_pos_(0) = odom->pose.pose.position.x;
	md_.laser_pos_(1) = odom->pose.pose.position.y;
	// 夹紧 Z 范围，防止 LIO 在坡道上 Z 发散 (RMUC 赛场高低差 < 1m)
	md_.laser_z_ = std::clamp(odom->pose.pose.position.z, -1.5, 1.5);
	md_.laser_q_ = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
									  odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
	md_.has_odom_ = true;
	// std::cout << "sdf_odom" << std::endl;
}

void SDFMap::laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser_msg)
{
	if (!md_.has_odom_)
	{
		return;
	}
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
	pcl::PointCloud<pcl::PointXY>
		latest_laser;

	pcl::fromROSMsg(*cloud_msg, latest_laser);
	md_.has_cloud_ = true;
	if (latest_laser.points.size() == 0)
		return;

	if (isnan(md_.laser_pos_(0)) || isnan(md_.laser_pos_(1)))
		return;

	this->resetBuffer(md_.laser_pos_ - mp_.local_update_range_,
					  md_.laser_pos_ + mp_.local_update_range_);

	pcl::PointXY pt;
	Eigen::Vector2d p2d, p2d_inf;

	int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);

	double max_x, max_y, min_x, min_y;

	min_x = mp_.map_max_boundary_(0);
	min_y = mp_.map_max_boundary_(1);

	max_x = mp_.map_min_boundary_(0);
	max_y = mp_.map_min_boundary_(1);
	for (size_t i = 0; i < latest_laser.points.size(); ++i)
	{
		pt = latest_laser.points[i];
		p2d(0) = pt.x, p2d(1) = pt.y;
		// p2d = R2d * p2d;

		Eigen::Vector2d devi = p2d - md_.laser_pos_;
		Eigen::Vector2i inf_pt;
		if (devi.norm() < 0.2)
			continue;
		if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1))
		{
			for (int x = -inf_step; x <= inf_step; ++x)
				for (int y = -inf_step; y <= inf_step; ++y)
				{

					p2d_inf(0) = pt.x + x * mp_.resolution_;
					p2d_inf(1) = pt.y + y * mp_.resolution_;

					max_x = max(max_x, p2d_inf(0));
					max_y = max(max_y, p2d_inf(1));

					min_x = min(min_x, p2d_inf(0));
					min_y = min(min_y, p2d_inf(1));

					posToIndex(p2d_inf, inf_pt);

					if (!isInMap(inf_pt))
						continue;

					int idx_inf = toAddress(inf_pt);

					md_.occupancy_buffer_inflate_[idx_inf] = 1;
				}
		}
	}
	min_x = min(min_x, md_.laser_pos_(0));
	min_y = min(min_y, md_.laser_pos_(1));

	max_x = max(max_x, md_.laser_pos_(0));
	max_y = max(max_y, md_.laser_pos_(1));

	posToIndex(mp_.local_update_range_ + md_.laser_pos_, md_.local_bound_max_);
	posToIndex(-mp_.local_update_range_ + md_.laser_pos_, md_.local_bound_min_);

	boundIndex(md_.local_bound_min_);
	boundIndex(md_.local_bound_max_);

	md_.esdf_need_update_ = true;
	md_.update_num_ += 1;
}
void SDFMap::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg)
{
	if (!md_.has_odom_)
		return;

	double min_h = node_->get_parameter("sdf_map.cloud_min_height").as_double();
	double max_h = node_->get_parameter("sdf_map.cloud_max_height").as_double();

	// 高度过滤相对机器人当前 Z 位置 (适应坡道/高低差地形)
	min_h += md_.laser_z_;
	max_h += md_.laser_z_;

	pcl::PointCloud<pcl::PointXYZ> cloud_3d;
	pcl::fromROSMsg(*cloud_msg, cloud_3d);
	md_.has_cloud_ = true;

	if (cloud_3d.points.empty())
		return;
	if (isnan(md_.laser_pos_(0)) || isnan(md_.laser_pos_(1)))
		return;

	RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
		"cloudCallback: %zu pts, robot at (%.2f, %.2f), height filter [%.2f, %.2f]",
		cloud_3d.points.size(), md_.laser_pos_(0), md_.laser_pos_(1), min_h, max_h);

	this->resetBuffer(md_.laser_pos_ - mp_.local_update_range_,
					  md_.laser_pos_ + mp_.local_update_range_);

	// 清空局部高程图
	Eigen::Vector2i local_min, local_max;
	posToIndex(-mp_.local_update_range_ + md_.laser_pos_, local_min);
	posToIndex(mp_.local_update_range_ + md_.laser_pos_, local_max);
	boundIndex(local_min);
	boundIndex(local_max);
	int buf_size = (int)md_.elevation_buffer_.size();
	for (int x = local_min(0); x <= local_max(0); ++x)
		for (int y = local_min(1); y <= local_max(1); ++y) {
			int addr = x * mp_.map_voxel_num_(1) + y;
			if (addr >= 0 && addr < buf_size)
				md_.elevation_buffer_[addr] = std::numeric_limits<float>::quiet_NaN();
		}

	// === Pass 1: 填充高程图 (每个格子取最低点作为地面高度) ===
	for (const auto &pt : cloud_3d.points)
	{
		if (pt.z < min_h - 0.5 || pt.z > max_h + 0.5)  // 宽松过滤，让高程图看到更多
			continue;

		Eigen::Vector2d p2d(pt.x, pt.y);
		Eigen::Vector2d devi = p2d - md_.laser_pos_;
		if (devi.norm() < 0.6)
			continue;
		if (fabs(devi(0)) >= mp_.local_update_range_(0) || fabs(devi(1)) >= mp_.local_update_range_(1))
			continue;

		Eigen::Vector2i idx;
		posToIndex(p2d, idx);
		if (!isInMap(idx))
			continue;

		int addr = toAddress(idx);
		if (addr < 0 || addr >= buf_size)
			continue;
		float &elev = md_.elevation_buffer_[addr];
		if (std::isnan(elev) || pt.z < elev)
			elev = pt.z;  // 取最低点作为地面
	}

	// === Pass 2: 根据坡度 + 高度判定障碍 ===
	int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
	int occupied_count = 0;

	for (const auto &pt : cloud_3d.points)
	{
		if (pt.z < min_h || pt.z > max_h)
			continue;

		Eigen::Vector2d p2d(pt.x, pt.y);
		Eigen::Vector2d devi = p2d - md_.laser_pos_;
		if (devi.norm() < 0.6)
			continue;
		if (fabs(devi(0)) >= mp_.local_update_range_(0) || fabs(devi(1)) >= mp_.local_update_range_(1))
			continue;

		Eigen::Vector2i idx;
		posToIndex(p2d, idx);
		if (!isInMap(idx))
			continue;

		// 计算该格子的局部坡度
		int addr = toAddress(idx);
		if (addr < 0 || addr >= buf_size)
			continue;
		float center_h = md_.elevation_buffer_[addr];
		if (std::isnan(center_h))
			continue;

		// 检查相邻格子的高度差，计算坡度
		bool is_slope_ok = true;
		float max_gradient = 0;
		for (int di = -1; di <= 1; ++di)
		{
			for (int dj = -1; dj <= 1; ++dj)
			{
				if (di == 0 && dj == 0) continue;
				Eigen::Vector2i nb(idx(0) + di, idx(1) + dj);
				if (!isInMap(nb)) continue;
				int nb_addr = toAddress(nb);
				if (nb_addr < 0 || nb_addr >= buf_size) continue;
				float nb_h = md_.elevation_buffer_[nb_addr];
				if (std::isnan(nb_h)) continue;

				float dh = fabs(center_h - nb_h);
				float dist = sqrt(di*di + dj*dj) * mp_.resolution_;
				float gradient = atan2(dh, dist);

				max_gradient = std::max(max_gradient, gradient);

				// 台阶检测: 单格高度差超过阈值 = 不可通行
				if (dh > mp_.step_height_max_)
					is_slope_ok = false;
			}
		}

		// 坡度超过最大可通行坡度 = 障碍
		if (max_gradient > mp_.max_slope_rad_)
			is_slope_ok = false;

		// 点远高于该格子的地面高度 = 悬空障碍 (墙壁/柱子)
		if (pt.z - center_h > 0.15)
			is_slope_ok = false;

		if (!is_slope_ok)
		{
			// 标记为障碍并膨胀
			for (int dx = -inf_step; dx <= inf_step; ++dx)
			{
				for (int dy = -inf_step; dy <= inf_step; ++dy)
				{
					Eigen::Vector2d p2d_inf(pt.x + dx * mp_.resolution_,
											pt.y + dy * mp_.resolution_);
					Eigen::Vector2i inf_pt;
					posToIndex(p2d_inf, inf_pt);
					if (!isInMap(inf_pt))
						continue;
					md_.occupancy_buffer_inflate_[toAddress(inf_pt)] = 1;
					occupied_count++;
				}
			}
		}
	}

	RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
		"Marked %d occupied cells (slope_max=%.0fdeg, step_max=%.2fm, inf_step=%d)",
		occupied_count, mp_.max_slope_rad_ * 180.0 / M_PI, mp_.step_height_max_, inf_step);

	Eigen::Vector2d bound_max_pos = mp_.local_update_range_ + md_.laser_pos_;
	Eigen::Vector2d bound_min_pos = -mp_.local_update_range_ + md_.laser_pos_;

	posToIndex(bound_max_pos, md_.local_bound_max_);
	posToIndex(bound_min_pos, md_.local_bound_min_);

	boundIndex(md_.local_bound_min_);
	boundIndex(md_.local_bound_max_);

	RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
		"cloudCB bounds: laser=(%.2f,%.2f) -> idx_min=(%d,%d) idx_max=(%d,%d)",
		md_.laser_pos_(0), md_.laser_pos_(1),
		md_.local_bound_min_(0), md_.local_bound_min_(1),
		md_.local_bound_max_(0), md_.local_bound_max_(1));

	md_.esdf_need_update_ = true;
	md_.update_num_ += 1;
}
void SDFMap::updateESDFCallback()
{
	if (!md_.esdf_need_update_)
		return;

	RCLCPP_INFO_ONCE(node_->get_logger(), "ESDF update starting...");
	auto t1 = rclcpp::Clock().now();
	updateESDF2d();
	auto t2 = rclcpp::Clock().now();
	RCLCPP_INFO_ONCE(node_->get_logger(), "ESDF update done in %.3fs", (t2 - t1).seconds());

	md_.esdf_time_ += (t2 - t1).seconds();
	md_.max_esdf_time_ = max(md_.max_esdf_time_, (t2 - t1).seconds());

	if (mp_.show_esdf_time_)
		RCLCPP_INFO(node_->get_logger(), "ESDF: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).seconds(),
					md_.esdf_time_ / md_.update_num_, md_.max_esdf_time_);

	md_.esdf_need_update_ = false;
}
template <typename F_get_val, typename F_set_val>
void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim)
{
	std::vector<int> v(mp_.map_voxel_num_(dim));
	std::vector<double> z(mp_.map_voxel_num_(dim) + 1);

	int k = start;
	v[start] = start;
	z[start] = -std::numeric_limits<double>::max();
	z[start + 1] = std::numeric_limits<double>::max();

	for (int q = start + 1; q <= end; q++)
	{
		k++;
		double s;

		do
		{
			k--;
			s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
		} while (s <= z[k]);

		k++;

		v[k] = q;
		z[k] = s;
		z[k + 1] = std::numeric_limits<double>::max();
	}

	k = start;

	for (int q = start; q <= end; q++)
	{
		while (z[k + 1] < q)
			k++;
		double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
		f_set_val(q, val);
	}
}
void SDFMap::updateESDF2d()
{
	Eigen::Vector2i min_esdf = md_.local_bound_min_;
	Eigen::Vector2i max_esdf = md_.local_bound_max_;

	/* ========== compute positive DT ========== */

	for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
	{
		fillESDF(
			[&](int y)
			{
				return md_.is_occupancy(toAddress(x, y)) == 1 ? 0 : std::numeric_limits<double>::max();
			},
			[&](int y, double val)
			{ md_.tmp_buffer1_[toAddress(x, y)] = val; }, min_esdf[1],
			max_esdf[1], 1);
	}
	for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
	{
		fillESDF(
			[&](int x)
			{
				return md_.tmp_buffer1_[toAddress(x, y)];
			},
			[&](int x, double val)
			{ md_.distance_buffer_[toAddress(x, y)] = sqrt(val) * mp_.resolution_; }, min_esdf[0],
			max_esdf[0], 0);
	}

	// /* ========== compute negative distance ========== */
	for (int x = min_esdf(0); x <= max_esdf(0); ++x)
		for (int y = min_esdf(1); y <= max_esdf(1); ++y)
		{
			int idx = toAddress(x, y);
			if (md_.is_occupancy(idx) == 0)
			{
				md_.occupancy_buffer_neg[idx] = 1;
			}
			else if (md_.is_occupancy(idx) == 1)
			{
				md_.occupancy_buffer_neg[idx] = 0;
			}
			else
			{
				RCLCPP_ERROR(node_->get_logger(), "what");
			}
		}

	for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
	{
		fillESDF(
			[&](int y)
			{
				return md_.occupancy_buffer_neg[toAddress(x, y)] == 1 ? 0 : std::numeric_limits<double>::max();
			},
			[&](int y, double val)
			{ md_.tmp_buffer1_[toAddress(x, y)] = val; }, min_esdf[1],
			max_esdf[1], 1);
	}
	for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
	{
		fillESDF(
			[&](int x)
			{
				return md_.tmp_buffer1_[toAddress(x, y)];
			},
			[&](int x, double val)
			{ md_.distance_buffer_neg_[toAddress(x, y)] = sqrt(val) * mp_.resolution_; }, min_esdf[0],
			max_esdf[0], 0);
	}

	/* ========== combine pos and neg DT ========== */
	for (int x = min_esdf(0); x <= max_esdf(0); ++x)
		for (int y = min_esdf(1); y <= max_esdf(1); ++y)
		{

			int idx = toAddress(x, y);
			md_.distance_buffer_all_[idx] = md_.distance_buffer_[idx];

			if (md_.distance_buffer_neg_[idx] > 0.0)
				md_.distance_buffer_all_[idx] += (-md_.distance_buffer_neg_[idx] + mp_.resolution_);
		}
}
void SDFMap::publishMap()
{
	pcl::PointXYZ pt;
	pcl::PointCloud<pcl::PointXYZ> cloud;

	Eigen::Vector2i min_cut = md_.local_bound_min_;
	Eigen::Vector2i max_cut = md_.local_bound_max_;

	int lmm = mp_.local_map_margin_ / 2;
	min_cut -= Eigen::Vector2i(lmm, lmm);
	max_cut += Eigen::Vector2i(lmm, lmm);

	boundIndex(min_cut);
	boundIndex(max_cut);

	for (int x = min_cut(0); x <= max_cut(0); ++x)
		for (int y = min_cut(1); y <= max_cut(1); ++y)
		{
			if (md_.is_occupancy(toAddress(x, y)) == 0)
				continue;
			
			Eigen::Vector2d pos;
			indexToPos(Eigen::Vector2i(x, y), pos);

			pt.x = pos(0);
			pt.y = pos(1);
			pt.z = 0;
			cloud.push_back(pt);
		}
	cloud.width = cloud.points.size();
	cloud.height = 1;
	cloud.is_dense = true;
	cloud.header.frame_id = mp_.frame_id_;
	sensor_msgs::msg::PointCloud2 cloud_msg;

	pcl::toROSMsg(cloud, cloud_msg);
	cloud_msg.header.stamp = node_->now();

	map_pub_->publish(cloud_msg);

	RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
		"publishMap: %zu pts, bounds=[%d,%d]->[%d,%d]",
		cloud.points.size(), min_cut(0), min_cut(1), max_cut(0), max_cut(1));
}
void SDFMap::getSurroundPts(const Eigen::Vector2d &pos, Eigen::Vector2d pts[2][2],
							Eigen::Vector2d &diff)
{
	if (!isInMap(pos))
	{
		// cout << "pos invalid for interpolation." << endl;
	}

	/* interpolation position */
	Eigen::Vector2d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector2d::Ones();
	Eigen::Vector2i idx;
	Eigen::Vector2d idx_pos;

	posToIndex(pos_m, idx);
	indexToPos(idx, idx_pos);
	diff = (pos - idx_pos) * mp_.resolution_inv_;

	for (int x = 0; x < 2; ++x)
	{
		for (int y = 0; y < 2; ++y)
		{
			Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
			Eigen::Vector2d current_pos;
			indexToPos(current_idx, current_pos);
			pts[x][y] = current_pos;
		}
	}
}
void SDFMap::publishESDF()
{
	double dist;
	pcl::PointCloud<pcl::PointXYZI> cloud;
	pcl::PointXYZI pt;

	const double min_dist = 0.0;
	const double max_dist = 3.0;

	// Only visualize within the actual ESDF computation range (no margin)
	// to avoid boundary artifacts (stale/invalid distance values outside)
	Eigen::Vector2i min_cut = md_.local_bound_min_;
	Eigen::Vector2i max_cut = md_.local_bound_max_;
	boundIndex(min_cut);
	boundIndex(max_cut);

	for (int x = min_cut(0); x <= max_cut(0); ++x)
		for (int y = min_cut(1); y <= max_cut(1); ++y)
		{

			Eigen::Vector2d pos;
			indexToPos(Eigen::Vector2i(x, y), pos);

			dist = getDistance(pos);
			dist = min(dist, max_dist);
			dist = max(dist, min_dist);

			pt.x = pos(0);
			pt.y = pos(1);
			pt.z = 0;
			// 反转: 障碍物附近 intensity 高(红色), 远离障碍 intensity 低(蓝色)
			pt.intensity = 1.0 - (dist - min_dist) / (max_dist - min_dist);
			cloud.push_back(pt);
		}

	cloud.width = cloud.points.size();
	cloud.height = 1;
	cloud.is_dense = true;
	cloud.header.frame_id = mp_.frame_id_;
	sensor_msgs::msg::PointCloud2 cloud_msg;
	pcl::toROSMsg(cloud, cloud_msg);
	cloud_msg.header.stamp = node_->now();

	esdf_pub_->publish(cloud_msg);

	if (!cloud.empty()) {
		Eigen::Vector2d first_pos, last_pos;
		indexToPos(Eigen::Vector2i(min_cut(0), min_cut(1)), first_pos);
		indexToPos(Eigen::Vector2i(max_cut(0), max_cut(1)), last_pos);
		RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
			"publishESDF: %zu pts, bounds=[%d,%d]->[%d,%d], world=(%.1f,%.1f)->(%.1f,%.1f)",
			cloud.points.size(), min_cut(0), min_cut(1), max_cut(0), max_cut(1),
			first_pos(0), first_pos(1), last_pos(0), last_pos(1));
	}
}
void SDFMap::visCallback()
{
	if (!md_.has_cloud_)
		return;
	publishMap();
	publish_map();
	publishESDF();

	// publishUnknown();
	// publishDepth();
}
double SDFMap::getResolution() { return mp_.resolution_; }
void SDFMap::getRegion(Eigen::Vector2d &ori, Eigen::Vector2d &size)
{
	ori = mp_.map_origin_, size = mp_.map_size_;
}

void SDFMap::posToIndex(const Eigen::Vector2d &pos, Eigen::Vector2i &id)
{
	for (int i = 0; i < 2; ++i)
		id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

void SDFMap::indexToPos(const Eigen::Vector2i &id, Eigen::Vector2d &pos)
{
	for (int i = 0; i < 2; ++i)
		pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

int SDFMap::toAddress(const Eigen::Vector2i &id)
{
	return id(0) * mp_.map_voxel_num_(1) + id(1);
}

int SDFMap::toAddress(int &x, int &y)
{
	return x * mp_.map_voxel_num_(1) + y;
}

void SDFMap::boundIndex(Eigen::Vector2i &id)
{
	Eigen::Vector2i id1;
	id1(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
	id1(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
	id = id1;
}

bool SDFMap::isInMap(const Eigen::Vector2i &idx)
{
	if (idx(0) < 0 || idx(1) < 0)
		return false;
	if (idx(0) > mp_.map_voxel_num_(0) - 1 || idx(1) > mp_.map_voxel_num_(1) - 1)
		return false;
	return true;
}

double SDFMap::getDistance(const Eigen::Vector2d &pos)
{
	Eigen::Vector2i id;
	posToIndex(pos, id);
	boundIndex(id);
	return md_.distance_buffer_all_[toAddress(id)];
}

bool SDFMap::isInMap(const Eigen::Vector2d &pos)
{
	if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4)
		return false;
	if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4)
		return false;
	return true;
}

int SDFMap::getInflateOccupancy(Eigen::Vector2d pos)
{
	if (!isInMap(pos))
		return -1;
	Eigen::Vector2i id;
	posToIndex(pos, id);
	return int(md_.is_occupancy(toAddress(id)));
}