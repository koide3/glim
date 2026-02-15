#pragma once

#include <deque>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>
#include <gtsam_points/util/runnning_statistics.hpp>

namespace glim {

class IMUValidation {
public:
  IMUValidation(const std::shared_ptr<spdlog::logger>& logger, bool enabled = true);
  ~IMUValidation();

  void validate(
    const Eigen::Isometry3d& last_T_world_imu,
    const Eigen::Vector3d& last_v_world_imu,
    const Eigen::Isometry3d& predicted_T_world_imu,
    const Eigen::Vector3d& predicted_v_world_imu,
    const Eigen::Isometry3d& corrected_T_world_imu,
    const Eigen::Vector3d& corrected_v_world_imu,
    double dt);

  void validate(const Eigen::Matrix<double, 6, 1>& imu_bias);

private:
  const bool enabled;
  std::shared_ptr<spdlog::logger> logger;

  int num_validations;
  gtsam_points::RunningStatistics<Eigen::Array3d> noimu_error_stats;  // rotation, translation, velocity
  gtsam_points::RunningStatistics<Eigen::Array3d> imu_error_stats;    // rotation, translation, velocity
  Eigen::Array3i imu_better_counts;                                   // rotation, translation, velocity

  int num_bias_validations;
  gtsam_points::RunningStatistics<Eigen::Array<double, 6, 1>> imu_bias_stats;  // Acc, Gyro bias
};

}  // namespace glim