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
class StreamTempBufferRoundRobin;
}  // namespace gtsam_ext

namespace glim {

class IMUIntegration;
class CloudDeskewing;
class CloudCovarianceEstimation;

/**
 * @brief GPU-based tightly coupled LiDAR-IMU frontend
 *
 */
class OdometryEstimation : public OdometryEstimationBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimation();
  virtual ~OdometryEstimation() override;

  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;
  virtual EstimationFrame::ConstPtr insert_frame(const PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) override;
  virtual std::vector<EstimationFrame::ConstPtr> get_remaining_frames() override;

private:
  void fallback_smoother();
  void update_frames(int current);
  void update_keyframes(int current);
  gtsam::NonlinearFactorGraph create_matching_cost_factors(int current);

private:
  std::string factor_type;

  double voxel_resolution;
  int max_num_keyframes;
  int full_connection_window_size;

  double keyframe_min_overlap;
  double keyframe_max_overlap;

  double smoother_lag;

  std::atomic_bool enable_matching_cost_factors;

  Eigen::Isometry3d T_lidar_imu;
  Eigen::Isometry3d T_imu_lidar;

  int marginalized_cursor;
  std::vector<EstimationFrame::Ptr> frames;
  std::vector<boost::shared_ptr<gtsam::ImuFactor>> imu_factors;

  std::vector<EstimationFrame::ConstPtr> keyframes;

  std::unique_ptr<InitialStateEstimation> init_estimation;

  std::unique_ptr<IMUIntegration> imu_integration;
  std::unique_ptr<CloudDeskewing> deskewing;
  std::unique_ptr<CloudCovarianceEstimation> covariance_estimation;

  std::unique_ptr<gtsam_ext::StreamTempBufferRoundRobin> stream_buffer_roundrobin;

  std::unique_ptr<gtsam_ext::IncrementalFixedLagSmootherExt> smoother;
};

}  // namespace glim
