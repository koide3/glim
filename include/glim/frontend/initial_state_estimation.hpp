#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/frontend/estimation_frame.hpp>

namespace glim {

/**
 * @brief Initial sensor state estimator
 *
 */
class InitialStateEstimation {
public:
  virtual ~InitialStateEstimation() {}

  /**
   * @brief Insert an IMU data
   * @param stamp         Timestamp
   * @param linear_acc    Linear acceleration
   * @param angular_vel   Angular velocity
   */
  virtual void insert_imu(double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {}

  /**
   * @brief Insert a point cloud
   * @param raw_frame Preprocessed point cloud
   */
  virtual void insert_frame(const PreprocessedFrame::ConstPtr& raw_frame) {}

  /**
   * @brief Get the initial estimation result
   * @return EstimationFrame::ConstPtr  Estimated initial state
   * @return nullptr                    If it's not ready
   */
  virtual EstimationFrame::ConstPtr initial_pose() = 0;
};

/**
 * @brief Naive initial state estimator that simply calculates a pose that aligns linear acc with the gravity direction
 *        Would not work well when the sensor is moving
 */
class NaiveInitialStateEstimation : public InitialStateEstimation {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  NaiveInitialStateEstimation();
  virtual ~NaiveInitialStateEstimation();

  virtual void insert_imu(double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;
  virtual EstimationFrame::ConstPtr initial_pose() override;

private:
  double stamp;
  Eigen::Vector3d sum_acc;

  Eigen::Matrix<double, 6, 1> imu_bias;
  Eigen::Isometry3d T_lidar_imu;
};

// TODO: Implement Loose-coupling-based initial state estimator

}  // namespace glim
