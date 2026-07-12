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
  bool show_esdf_time_, show_occ_time_;

  // 法向量坡度判定参数
  double max_slope_rad_;        // 平均 |n.z| < cos(此角) 判障碍
  double cloud_min_h_ = -0.2;   // z 带下沿 (相对机身 z)
  double cloud_max_h_ = 0.15;   // z 带上沿 (相对机身 z)：只看机身高度附近的表面
  double normal_voxel_leaf_ = 0.08; // 法向量前的体素降采样 (m)
  int normal_k_ = 10;           // kNN 邻域点数
  int normal_count_thresh_ = 3; // 每格最少支持点数，防稀疏噪声成障

  // Log-odds 贝叶斯占据更新参数
  double logodds_hit_, logodds_miss_;
  double logodds_max_, logodds_min_;
  double logodds_thresh_;

  // 占据超时 (s)：超过此时长未被命中的格子视为空闲，<=0 关闭。
  // 自转下点云 yaw 抖动会把命中涂抹到邻格形成幽灵障碍，raycast 只能清除
  // 恰好被后续射线穿过的格子；超时兜底让未再命中的幽灵自动过期。
  double occ_timeout_ = 0.0;

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

  // Log-odds 累积值，0 = 未知
  std::vector<float> logodds_buffer_;

  // 每格最近一次命中时刻 (s)，未命中过 = kNeverHit；配合 occ_timeout_ 使用
  static constexpr float kNeverHit = -1e9f;
  std::vector<float> last_hit_time_;

  // 静态先验层：确定性不可通行基准。动态点云层只能在其上叠加临时障碍，
  // 永远不能清除静态格（不受 raycast / occ_timeout 影响）。
  // 存储 [x * static_h_ + y]，与全局规划器 PriorMap 同约定 (y 已翻转到世界系向上)。
  bool has_static_ = false;
  std::vector<uint8_t> static_map_;
  Eigen::Vector2d static_origin_ = Eigen::Vector2d::Zero();
  double static_res_inv_ = 20.0;
  int static_w_ = 0, static_h_ = 0;

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
