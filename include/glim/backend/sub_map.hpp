#pragma once

#include <memory>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/frontend/estimation_frame.hpp>

namespace  glim {

struct SubMap {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<SubMap>;
  using ConstPtr = std::shared_ptr<const SubMap>;

public:
  int id;
  
  Eigen::Isometry3d T_world_origin;
  Eigen::Isometry3d T_origin_endpoint_L;
  Eigen::Isometry3d T_origin_endpoint_R;

  gtsam_ext::VoxelizedFrame::ConstPtr frame;
  std::vector<EstimationFrame::ConstPtr> frames;
  std::vector<EstimationFrame::ConstPtr> odom_frames;
};

} // namespace  glim 
