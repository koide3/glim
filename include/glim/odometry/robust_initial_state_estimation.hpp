#pragma once

#include <memory>
#include <glim/odometry/initial_state_estimation.hpp>

namespace gtsam {
class Values;
}

namespace gtsam_points {
struct FlatContainer;
template <typename VoxelContents>
class IncrementalVoxelMap;
using iVox = IncrementalVoxelMap<FlatContainer>;
}  // namespace gtsam_points

namespace glim {

class IMUIntegration;
class CloudDeskewing;
class CloudCovarianceEstimation;

/**
 * @brief Robust initial state estimation.
 *        This class first performs odometry estimation using GICP frame-to-model registration with
 *        angular velocity-based pose prediction and deskewing.
 *        Then, it performs the IMU angular velocity bias estimation followed by the linear estimation
 *        of the gravity, velocity, and bias.
 *        Finally, optionally, it refines the estimation using nonlinear optimization (but we found it
 *        degrades the estimation, and thus turned it off).
 */
class RobustInitialStateEstimation : public InitialStateEstimation {
public:
  RobustInitialStateEstimation(const Eigen::Isometry3d& T_lidar_imu);
  virtual ~RobustInitialStateEstimation() override;

  virtual void insert_imu(double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;
  virtual void insert_frame(const PreprocessedFrame::ConstPtr& raw_frame) override;
  virtual EstimationFrame::ConstPtr initial_pose() override;

  void enable_nonlinear_refinement();

private:
  void update_odom(const PreprocessedFrame::ConstPtr& raw_frame);
  gtsam::Values refine_nonlinear(const gtsam::Values& gravity_aligned_values);

  Eigen::Vector3d estimate_gyro_bias();
  gtsam::Values estimate_linear_system(const Eigen::Vector3d& gyro_bias);

private:
  const Eigen::Isometry3d T_lidar_imu;

  int num_threads;
  double window_size;
  bool use_nonlinear_refinement;

  std::unique_ptr<CloudDeskewing> deskewing;
  std::unique_ptr<CloudCovarianceEstimation> covariance_estimation;
  std::unique_ptr<IMUIntegration> imu_integration;

  std::shared_ptr<gtsam_points::iVox> target_ivox;
  std::vector<std::pair<double, Eigen::Isometry3d>> Ts_odom_lidar;
};

}  // namespace glim
