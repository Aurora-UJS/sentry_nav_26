#pragma once
/**
 * map_data.hpp: 地图数据结构体定义
 *
 * MappingParameters — 地图配置参数
 * MappingData       — 运行时地图缓冲区
 * Global_Map        — 全局先验地图数据
 */

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <vector>
#include <string>
#include <limits>

struct Global_Map
{
  std::string path;
  Eigen::Vector2d map_origin_, map_size_;
  Eigen::Vector2d image_size;
  double resolution_, resolution_inv_;
  std::vector<char> occupancy_buffer_inflate_Global_Map;
  Eigen::Vector2i map_voxel_num_;
};

struct MappingParameters
{
  Eigen::Vector2d map_origin_, map_size_;
  Eigen::Vector2d image_size;
  Eigen::Vector2d map_min_boundary_, map_max_boundary_;
  Eigen::Vector2d local_update_range_;
  int local_map_margin_;

  Eigen::Vector2i map_voxel_num_;

  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  double max_slope_rad_;
  double step_height_max_;
  bool show_esdf_time_, show_occ_time_;

  // Log-odds 贝叶斯占据更新参数
  double logodds_hit_, logodds_miss_;
  double logodds_max_, logodds_min_;
  double logodds_thresh_;

  std::string frame_id_;
};

struct MappingData
{
  Eigen::Vector2d laser_pos_, last_laser_pos_;
  Eigen::Quaterniond laser_q_, last_laser_q_;
  double laser_z_ = 0.0;

  bool has_odom_, has_cloud_;
  std::vector<char> occupancy_buffer_inflate_;
  std::vector<Global_Map> Global_Maps;
  std::vector<char> occupancy_buffer_neg;
  std::vector<double> distance_buffer_neg_;
  std::vector<double> distance_buffer_all_;

  std::vector<double> distance_buffer_;

  std::vector<float> elevation_buffer_;
  std::vector<char> slope_obstacle_buffer_;

  // Log-odds 累积值，0 = 未知
  std::vector<float> logodds_buffer_;

  std::vector<double> tmp_buffer1_;
  Eigen::Vector2i local_bound_min_, local_bound_max_;
  Eigen::Vector2i ring_offset_;
  bool local_updated_, esdf_need_update_;
  double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
  int update_num_;
  bool use_global_map = false;
  int global_map_num = 0;
  int current_global_map = 0;
  inline int is_occupancy(int idx)
  {
    return occupancy_buffer_inflate_[idx] || (use_global_map ? Global_Maps[current_global_map].occupancy_buffer_inflate_Global_Map[idx] : 0);
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
