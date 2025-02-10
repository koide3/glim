#pragma once

#include <deque>
#include <vector>
#include <Eigen/Core>
#include <gtsam/navigation/ImuFactor.h>

namespace glim {

/**
 * @brief IMU integration parameters
 */
struct IMUIntegrationParams {
  IMUIntegrationParams(const bool upright = true);
  ~IMUIntegrationParams();

  bool upright;       // If true, +Z = up
  double acc_noise;   // Linear acceleration noise
  double gyro_noise;  // Angular velocity noise
  double int_noise;   // Integration noise
};

/**
 * @brief Utility class to integrate IMU measurements
 */
class IMUIntegration {
public:
  IMUIntegration(const IMUIntegrationParams& params = IMUIntegrationParams());
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
    std::vector<Eigen::Isometry3d>& pred_poses);

  /**
   * @brief Find IMU data in a time range
   * @param start_time  Start time
   * @param end_time    End time
   * @param delta_times Delta times (interval between IMU frames)
   * @param imu_data    IMU data
   * @return Index of the last integrated IMU frame
   */
  int find_imu_data(double start_time, double end_time, std::vector<double>& delta_times, std::vector<Eigen::Matrix<double, 7, 1>>& imu_data);

  /**
   * @brief Erase IMU data before the given index
   * @param last Last integrated IMU measurement index
   */
  void erase_imu_data(int last);

  /**
   * @brief Preintegrated measurements
   */
  const gtsam::PreintegratedImuMeasurements& integrated_measurements() const;

  /**
   * @brief IMU data in queue
   */
  const std::deque<Eigen::Matrix<double, 7, 1>>& imu_data_in_queue() const;

private:
  std::shared_ptr<gtsam::PreintegratedImuMeasurements> imu_measurements;
  std::deque<Eigen::Matrix<double, 7, 1>> imu_queue;
};
}  // namespace glim