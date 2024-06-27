#pragma once

#include <memory>
#include <glim/odometry/initial_state_estimation.hpp>

namespace gtsam_points {
struct FlatContainer;
template <typename VoxelContents>
class IncrementalVoxelMap;
using iVox = IncrementalVoxelMap<FlatContainer>;
}  // namespace gtsam_points

namespace glim {

class IMUIntegration;
class CloudCovarianceEstimation;

class LooseInitialStateEstimation : public InitialStateEstimation {
public:
  LooseInitialStateEstimation(const Eigen::Isometry3d& T_lidar_imu, const Eigen::Matrix<double, 6, 1>& imu_bias);
  virtual ~LooseInitialStateEstimation() override;

  virtual void insert_frame(const PreprocessedFrame::ConstPtr& raw_frame) override;
  virtual void insert_imu(double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;
  virtual EstimationFrame::ConstPtr initial_pose() override;

private:
  const Eigen::Isometry3d T_lidar_imu;

  int num_threads;
  double window_size;

  std::unique_ptr<CloudCovarianceEstimation> covariance_estimation;

  std::shared_ptr<gtsam_points::iVox> target_ivox;
  std::vector<std::pair<double, Eigen::Isometry3d>> T_odom_lidar;

  std::unique_ptr<IMUIntegration> imu_integration;
};

}  // namespace glim
