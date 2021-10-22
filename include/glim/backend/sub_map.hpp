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

  void drop_odom_frames() {
    for (auto& frame : frames) {
      frame = frame->clone_wo_points();
    }

    for (auto& frame : odom_frames) {
      frame = frame->clone_wo_points();
    }
  }

public:
  int id;

  Eigen::Isometry3d T_world_origin;
  Eigen::Isometry3d T_origin_endpoint_L;
  Eigen::Isometry3d T_origin_endpoint_R;

  gtsam_ext::VoxelizedFrame::ConstPtr frame;
  std::vector<EstimationFrame::ConstPtr> frames;
  std::vector<EstimationFrame::ConstPtr> odom_frames;
};

}  // namespace  glim
