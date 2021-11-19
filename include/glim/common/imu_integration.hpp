#pragma once

#include <deque>
#include <vector>
#include <Eigen/Core>
#include <gtsam/navigation/ImuFactor.h>

namespace glim {

class IMUIntegration {
public:
  IMUIntegration();
  ~IMUIntegration();

  void insert_imu(double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);

  int integrate_imu(double start_time, double end_time, const gtsam::imuBias::ConstantBias& bias, int* num_integrated);

  int integrate_imu(
    double start_time,
    double end_time,
    const gtsam::NavState& state,
    const gtsam::imuBias::ConstantBias& bias,
    std::vector<double>& pred_times,
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& pred_poses);

  int integrate_imu(
    double start_time,
    double end_time,
    const gtsam::NavState& state,
    const gtsam::imuBias::ConstantBias& bias,
    std::vector<double>& pred_times,
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& pred_poses,
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& pred_vels,
    std::vector<Eigen::Matrix<double, 7, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 7, 1>>>& measurements);

  void erase_imu_data(int last);

  const gtsam::PreintegratedImuMeasurements& integrated_measurements() const;

private:
  std::shared_ptr<gtsam::PreintegratedImuMeasurements> imu_measurements;
  std::deque<Eigen::Matrix<double, 7, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 7, 1>>> imu_queue;
};
}  // namespace glim