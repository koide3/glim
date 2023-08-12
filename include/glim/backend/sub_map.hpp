#pragma once

#include <memory>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/frontend/estimation_frame.hpp>

namespace glim {

/**
 * @brief SubMap
 *
 */
struct SubMap {
public:
  using Ptr = std::shared_ptr<SubMap>;
  using ConstPtr = std::shared_ptr<const SubMap>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Remove point clouds of the odometry estimation frames (to save memory)
   */
  void drop_frame_points();

  /**
   * @brief Save the submap
   * @param path  Save path
   */
  void save(const std::string& path) const;

  /**
   * @brief  Load a submap from storage
   * @param path    Load path
   * @return SubMap::Ptr Loaded SubMap
   * @return nullptr if failed to load
   */
  static SubMap::Ptr load(const std::string& path);

public:
  int id;  ///< submap ID

  Eigen::Isometry3d T_world_origin;       ///< frame[frame.size() / 2] pose w.r.t. the world
  Eigen::Isometry3d T_origin_endpoint_L;  ///< frame.front() pose w.r.t. the origin
  Eigen::Isometry3d T_origin_endpoint_R;  ///< frame.back() pose w.r.t. the origin

  gtsam_ext::PointCloud::Ptr frame;                              ///< Merged submap frame
  std::vector<gtsam_ext::GaussianVoxelMap::Ptr> voxelmaps;  ///< Multi-resolution voxelmaps

  std::vector<EstimationFrame::ConstPtr> frames;       ///< Optimized odometry frames
  std::vector<EstimationFrame::ConstPtr> odom_frames;  ///< Original odometry frames
};

}  // namespace  glim
