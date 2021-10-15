#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/frontend/estimation_frame.hpp>

namespace glim {

class InitialStateEstimation {
public:
  virtual ~InitialStateEstimation() {}

  virtual void insert_imu(double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {}
  virtual void insert_frame(const PreprocessedFrame::ConstPtr& raw_frame) {}

  virtual EstimationFrame::ConstPtr initial_pose() = 0;
};

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

}  // namespace glim
