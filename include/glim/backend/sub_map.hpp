#pragma once

#include <memory>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/frontend/estimation_frame.hpp>

namespace glim {

struct SubMap {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<SubMap>;
  using ConstPtr = std::shared_ptr<const SubMap>;

  void drop_odom_frames();

  void save(const std::string& path);
  static SubMap::Ptr load(const std::string& path);

public:
  int id;

  Eigen::Isometry3d T_world_origin;       // frame[frame.size() / 2] pose w.r.t. the world
  Eigen::Isometry3d T_origin_endpoint_L;  // frame.front() pose w.r.t. the origin
  Eigen::Isometry3d T_origin_endpoint_R;  // frame.back() pose w.r.t. the origin

  gtsam_ext::Frame::Ptr frame;                         // Merged submap frame
  std::vector<EstimationFrame::ConstPtr> frames;       // Optimized odometry frames
  std::vector<EstimationFrame::ConstPtr> odom_frames;  // Original odometry frames
};

}  // namespace  glim
