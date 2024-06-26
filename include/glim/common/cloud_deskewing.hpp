#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glim {

/**
 * @brief Dewsking point cloud
 */
class CloudDeskewing {
public:
  CloudDeskewing();
  ~CloudDeskewing();

  /**
   * @brief Deskew a point cloud with a constant linear and angular velocity assumption
   * @param T_imu_lidar  Transformation between IMU and LiDAR
   * @param linear_vel   IMU linear velocity (v_world_imu)
   * @param angular_vel  IMU angular velocity (omega_world_imu)
   * @param times        Per-point timestamps
   * @param points       Input points
   * @return Deskewed points
   */
  std::vector<Eigen::Vector4d> deskew(
    const Eigen::Isometry3d& T_imu_lidar,
    const Eigen::Vector3d& linear_vel,
    const Eigen::Vector3d& angular_vel,
    const std::vector<double>& times,
    const std::vector<Eigen::Vector4d>& points);

  /**
   * @brief Deskew a point cloud with IMU pose prediction
   * @param T_imu_lidar  Transformation between IMU and LiDAR
   * @param imu_times    Timestamps of IMU pose prediction (should cover the time range of $times$)
   * @param imu_poses    Predicted IMU poses (T_world_imu)
   * @param stamp        Timestamp of the first point
   * @param times        Per-point timestamps with respect to the first one
   * @param points       Input points
   * @return Deskewed points
   */
  std::vector<Eigen::Vector4d> deskew(
    const Eigen::Isometry3d& T_imu_lidar,
    const std::vector<double>& imu_times,
    const std::vector<Eigen::Isometry3d>& imu_poses,
    const double stamp,
    const std::vector<double>& times,
    const std::vector<Eigen::Vector4d>& points);
};

}  // namespace glim