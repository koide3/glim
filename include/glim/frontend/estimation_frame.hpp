#pragma once

#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam_ext/types/voxelized_frame.hpp>
#include <glim/preprocess/preprocessed_frame.hpp>

namespace glim {

enum class FrameID { WORLD, LIDAR, IMU };

/**
 * @brief Odometry estimation frame
 *
 */
struct EstimationFrame {
  using Ptr = std::shared_ptr<EstimationFrame>;
  using ConstPtr = std::shared_ptr<const EstimationFrame>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Make a clone of the estimation frame instance but without points data
   * @return EstimationFrame::Ptr   Cloned frame without points
   */
  EstimationFrame::Ptr clone_wo_points() const {
    EstimationFrame::Ptr cloned(new EstimationFrame);
    *cloned = *this;
    cloned->raw_frame.reset();
    cloned->frame.reset();
    cloned->voxelmaps.clear();
    return cloned;
  }

  /**
   * @brief Get the sensor pose according to frame_id
   * @return const Eigen::Isometry3d  Sensor pose
   */
  const Eigen::Isometry3d T_world_sensor() const {
    switch (frame_id) {
      case FrameID::WORLD:
        return Eigen::Isometry3d::Identity();
      case FrameID::LIDAR:
        return T_world_lidar;
      case FrameID::IMU:
        return T_world_imu;
    }
    return Eigen::Isometry3d::Identity();
  }

  /**
   * @brief Set the sensor pose
   * @param frame_id  Sensor coodinate frame
   * @param T         Sensor pose
   */
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

public:
  long id;       ///< Frame ID
  double stamp;  ///< Timestamp

  Eigen::Isometry3d T_lidar_imu;    ///< LiDAR-IMU transformation
  Eigen::Isometry3d T_world_lidar;  ///< LiDAR pose in the world space
  Eigen::Isometry3d T_world_imu;    ///< IMU pose in the world space

  Eigen::Vector3d v_world_imu;           ///< IMU velocity in the world frame
  Eigen::Matrix<double, 6, 1> imu_bias;  ///< IMU bias

  PreprocessedFrame::ConstPtr raw_frame;             ///< Raw input point cloud
  Eigen::Matrix<double, 8, -1> imu_rate_trajectory;  ///< IMU-rate trajectory 8 x N  [t, x, y, z, qx, qy, qz, qw]

  FrameID frame_id;                  ///< Coordinate frame of $frame
  gtsam_ext::Frame::ConstPtr frame;  ///< Deskewed points for state estimation

  std::vector<gtsam_ext::GaussianVoxelMap::Ptr> voxelmaps;  ///< Multi-resolution voxelmaps
};
}  // namespace glim