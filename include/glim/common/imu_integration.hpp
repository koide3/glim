#pragma once

#include <deque>
#include <vector>
#include <Eigen/Core>
#include <gtsam/navigation/ImuFactor.h>

namespace glim {

/**
 * @brief Utility class to integrate IMU measurements
 */
class IMUIntegration {
public:
  IMUIntegration();
  ~IMUIntegration();

  /**
   * @brief Insert an IMU data
   * @param stamp       Timestamp
   * @param linear_acc  Linear acceleration
   * @param angular_vel Angular velocity
   */
  void insert_imu(double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);

  /**
   * @brief Integrate IMU measurements in a time range
   * @param start_time     Integration starting time
   * @param end_time       Integration ending time
   * @param bias           IMU bias
   * @param num_integrated Number of integrated IMU measurements
   * @return Index of the last integrated IMU frame
   */
  int integrate_imu(double start_time, double end_time, const gtsam::imuBias::ConstantBias& bias, int* num_integrated);

  /**
   * @brief Integrate IMU measurements and predict IMU poses in a time range
   * @param start_time     Integration starting time
   * @param end_time       Integration ending time
   * @param state          IMU NavState
   * @param bias           IMU bias
   * @param pred_times     Timestamps of predicted IMU poses
   * @param pred_poses     Predicted IMU poses
   * @return Index of the last integrated IMU frame
   */
  int integrate_imu(
    double start_time,
    double end_time,
    const gtsam::NavState& state,
    const gtsam::imuBias::ConstantBias& bias,
    std::vector<double>& pred_times,
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& pred_poses);

  /**
   * @brief Integrate IMU measurements and predict IMU poses in a time range
   * @param start_time     Integration starting time
   * @param end_time       Integration ending time
   * @param state          IMU NavState
   * @param bias           IMU bias
   * @param pred_times     Timestamps of predicted IMU poses
   * @param pred_poses     Predicted IMU poses
   * @param pred_vels      Predicted IMU velocities
   * @param measurements   Integrated IMU measurements [t, ax, ay, az, wx, wy, wz]
   * @return Index of the last integrated IMU frame
   */
  int integrate_imu(
    double start_time,
    double end_time,
    const gtsam::NavState& state,
    const gtsam::imuBias::ConstantBias& bias,
    std::vector<double>& pred_times,
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& pred_poses,
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& pred_vels,
    std::vector<Eigen::Matrix<double, 7, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 7, 1>>>& measurements);

  /**
   * @brief Erase IMU data before the given index
   * @param last Last integrated IMU measurement index
   */
  void erase_imu_data(int last);

  /**
   * @brief Preintegrated measurements
   */
  const gtsam::PreintegratedImuMeasurements& integrated_measurements() const;

private:
  std::shared_ptr<gtsam::PreintegratedImuMeasurements> imu_measurements;
  std::deque<Eigen::Matrix<double, 7, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 7, 1>>> imu_queue;
};
}  // namespace glim