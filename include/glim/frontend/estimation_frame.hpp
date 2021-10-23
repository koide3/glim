#pragma once

#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam_ext/types/voxelized_frame.hpp>
#include <glim/preprocess/preprocessed_frame.hpp>

namespace glim {

enum class FrameID { WORLD, LIDAR, IMU };

struct EstimationFrame {
  using Ptr = std::shared_ptr<EstimationFrame>;
  using ConstPtr = std::shared_ptr<const EstimationFrame>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EstimationFrame::Ptr clone_wo_points() const {
    EstimationFrame::Ptr cloned(new EstimationFrame);
    *cloned = *this;
    cloned->raw_frame.reset();
    cloned->frame.reset();
    return cloned;
  }

  gtsam_ext::VoxelizedFrame::ConstPtr voxelized_frame() const { return std::dynamic_pointer_cast<const gtsam_ext::VoxelizedFrame>(frame); }

  const Eigen::Isometry3d T_world_sensor() const {
    switch (frame_id) {
      case FrameID::WORLD:
        return Eigen::Isometry3d::Identity();
      case FrameID::LIDAR:
        return T_world_lidar;
      case FrameID::IMU:
        return T_world_imu;
    }
  }

  void set_T_world_sensor(FrameID frame_id, const Eigen::Isometry3d& T) {
    switch (frame_id) {
      default:
        std::cerr << "error: frame_id must be either of LIDAR or IMU" << std::endl;
        abort();
        break;

      case FrameID::LIDAR:
        T_world_lidar = T;
        T_world_imu = T_world_lidar * T_lidar_imu;
        break;

      case FrameID::IMU:
        T_world_imu = T;
        T_world_lidar = T_world_imu * T_lidar_imu.inverse();
        break;
    }
  }

  long id;
  double stamp;

  Eigen::Isometry3d T_lidar_imu;
  Eigen::Isometry3d T_world_lidar;
  Eigen::Isometry3d T_world_imu;

  Eigen::Vector3d v_world_imu;
  Eigen::Matrix<double, 6, 1> imu_bias;

  PreprocessedFrame::ConstPtr raw_frame;

  FrameID frame_id;  // "lidar" or "imu"
  gtsam_ext::Frame::ConstPtr frame;
};
}  // namespace glim