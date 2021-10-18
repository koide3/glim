#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glim {

class CloudDeskewing {
public:
  CloudDeskewing();
  ~CloudDeskewing();

  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> deskew(
    const Eigen::Isometry3d& T_imu_lidar,
    const Eigen::Vector3d& linear_vel,
    const Eigen::Vector3d& angular_vel,
    const std::vector<double>& times,
    const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& points);

  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> deskew(
    const Eigen::Isometry3d& T_imu_lidar,
    const std::vector<double>& imu_times,
    const std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& imu_poses,
    const double stamp,
    const std::vector<double>& times,
    const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& points);
};

}  // namespace glim