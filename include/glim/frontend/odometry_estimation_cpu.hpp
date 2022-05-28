#pragma once

#include <memory>
#include <random>

#include <boost/shared_ptr.hpp>
#include <glim/frontend/odometry_estimation_base.hpp>
#include <glim/frontend/initial_state_estimation.hpp>

namespace gtsam {
class ImuFactor;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace gtsam_ext {
class iVox;
class GaussianVoxelMapCPU;
class IncrementalFixedLagSmootherExt;
class IncrementalFixedLagSmootherExtWithFallback;
}  // namespace gtsam_ext

namespace glim {

class IMUIntegration;
class CloudDeskewing;
class CloudCovarianceEstimation;

struct OdometryEstimationCPUParams {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationCPUParams();
  ~OdometryEstimationCPUParams();

  enum class KeyframeUpdateStrategy { OVERLAP, DISPLACEMENT, ENTROPY };

public:
  // Sensor params;
  bool fix_imu_bias;
  Eigen::Isometry3d T_lidar_imu;
  Eigen::Matrix<double, 6, 1> imu_bias;

  // Init state
  bool estimate_init_state;
  Eigen::Isometry3d init_T_world_imu;
  Eigen::Vector3d init_v_world_imu;

  std::string registration_type;
  int max_iterations;

  int lru_thresh;
  double target_downsampling_rate;

  double ivox_resolution;
  double ivox_min_dist;

  double vgicp_resolution;
  int vgicp_voxelmap_levels;
  double vgicp_voxelmap_scaling_factor;

  // Optimization params
  double smoother_lag;
  bool use_isam2_dogleg;
  double isam2_relinearize_skip;
  double isam2_relinearize_thresh;

  // Logging params
  bool save_imu_rate_trajectory;

  int num_threads;
};

class OdometryEstimationCPU : public OdometryEstimationBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationCPU(const OdometryEstimationCPUParams& params = OdometryEstimationCPUParams());
  virtual ~OdometryEstimationCPU() override;

  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;
  virtual EstimationFrame::ConstPtr insert_frame(const PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) override;
  virtual std::vector<EstimationFrame::ConstPtr> get_remaining_frames() override;

private:
  gtsam::NonlinearFactorGraph create_factors(const int current, const boost::shared_ptr<gtsam::ImuFactor>& imu_factor);

  void update_target(const Eigen::Isometry3d& T_world_frame, const gtsam_ext::Frame::ConstPtr& frame);
  void update_frames(const int current);

private:
  using Params = OdometryEstimationCPUParams;
  Params params;

  // Sensor extrinsic params
  Eigen::Isometry3d T_lidar_imu;
  Eigen::Isometry3d T_imu_lidar;

  // Frames & keyframes
  int marginalized_cursor;
  std::vector<EstimationFrame::Ptr> frames;

  std::mt19937 mt;
  Eigen::Isometry3d target_updated_pose;
  std::vector<std::shared_ptr<gtsam_ext::GaussianVoxelMapCPU>> target_voxelmaps;
  std::shared_ptr<gtsam_ext::iVox> target_ivox;
  EstimationFrame::ConstPtr target_ivox_frame;

  // Utility classes
  std::unique_ptr<InitialStateEstimation> init_estimation;
  std::unique_ptr<IMUIntegration> imu_integration;
  std::unique_ptr<CloudDeskewing> deskewing;
  std::unique_ptr<CloudCovarianceEstimation> covariance_estimation;

  // Optimizer
  using FixedLagSmootherExt = gtsam_ext::IncrementalFixedLagSmootherExtWithFallback;
  std::unique_ptr<FixedLagSmootherExt> smoother;
};

}  // namespace glim
