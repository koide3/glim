#pragma once

#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam_ext/types/voxelized_frame.hpp>
#include <glim/preprocess/preprocessed_frame.hpp>

namespace glim {

struct EstimationFrame {
  using Ptr = std::shared_ptr<EstimationFrame>;
  using ConstPtr = std::shared_ptr<const EstimationFrame>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void purge_frames() const {
    // warning: dirty code!!
    std::const_pointer_cast<PreprocessedFrame>(raw_frame).reset();
    std::const_pointer_cast<gtsam_ext::VoxelizedFrame>(frame).reset();
  }

  long id;
  double stamp;

  Eigen::Isometry3d T_lidar_imu;
  Eigen::Isometry3d T_world_lidar;
  Eigen::Isometry3d T_world_imu;

  Eigen::Vector3d v_world_imu;
  Eigen::Matrix<double, 6, 1> imu_bias;

  PreprocessedFrame::ConstPtr raw_frame;

  std::string frame_id;  // "lidar" or "imu"
  gtsam_ext::VoxelizedFrame::ConstPtr frame;
};
}  // namespace glim