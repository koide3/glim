#pragma once

#include <memory>

#include <boost/shared_ptr.hpp>
#include <glim/frontend/odometry_estimation_base.hpp>
#include <glim/frontend/initial_state_estimation.hpp>

namespace gtsam {
class ImuFactor;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace gtsam_ext {
class VoxelizedFrame;
class IncrementalFixedLagSmootherExt;
class IncrementalFixedLagSmootherExtWithFallback;
class StreamTempBufferRoundRobin;
class CUDAStream;
}  // namespace gtsam_ext

namespace glim {

class IMUIntegration;
class CloudDeskewing;
class CloudCovarianceEstimation;

struct OdometryEstimationGPUParams {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationGPUParams();
  ~OdometryEstimationGPUParams();

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

  // Optimization params
  double smoother_lag;
  bool use_isam2_dogleg;
  double isam2_relinearize_skip;
  double isam2_relinearize_thresh;

  double voxel_resolution;
  int voxelmap_levels;
  double voxelmap_scaling_factor;

  int max_num_keyframes;
  int full_connection_window_size;

  // Keyframe management params
  KeyframeUpdateStrategy keyframe_strategy;
  double keyframe_min_overlap;
  double keyframe_max_overlap;
  double keyframe_delta_trans;
  double keyframe_delta_rot;
  double keyframe_entropy_thresh;

  // Logging params
  bool save_imu_rate_trajectory;
};

/**
 * @brief GPU-based tightly coupled LiDAR-IMU frontend
 *
 */
class OdometryEstimationGPU : public OdometryEstimationBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationGPU(const OdometryEstimationGPUParams& params = OdometryEstimationGPUParams());
  virtual ~OdometryEstimationGPU() override;

  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;
  virtual EstimationFrame::ConstPtr insert_frame(const PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) override;
  virtual std::vector<EstimationFrame::ConstPtr> get_remaining_frames() override;

private:
  gtsam::NonlinearFactorGraph create_matching_cost_factors(int current);

  void fallback_smoother();
  void update_frames(int current);
  void update_keyframes_overlap(int current);
  void update_keyframes_displacement(int current);
  void update_keyframes_entropy(const gtsam::NonlinearFactorGraph& matching_cost_factors, int current);

private:
  using Params = OdometryEstimationGPUParams;
  Params params;

  int entropy_num_frames;
  double entropy_running_average;

  // Sensor extrinsic params
  Eigen::Isometry3d T_lidar_imu;
  Eigen::Isometry3d T_imu_lidar;

  // Frames & keyframes
  int marginalized_cursor;
  std::vector<EstimationFrame::Ptr> frames;
  std::vector<boost::shared_ptr<gtsam::ImuFactor>> imu_factors;

  std::vector<EstimationFrame::ConstPtr> keyframes;

  // Utility classes
  std::unique_ptr<InitialStateEstimation> init_estimation;
  std::unique_ptr<IMUIntegration> imu_integration;
  std::unique_ptr<CloudDeskewing> deskewing;
  std::unique_ptr<CloudCovarianceEstimation> covariance_estimation;

  std::unique_ptr<gtsam_ext::CUDAStream> stream;
  std::unique_ptr<gtsam_ext::StreamTempBufferRoundRobin> stream_buffer_roundrobin;

  // Optimizer
  using FixedLagSmootherExt = gtsam_ext::IncrementalFixedLagSmootherExtWithFallback;
  std::unique_ptr<FixedLagSmootherExt> smoother;
};

}  // namespace glim
