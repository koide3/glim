#pragma once

#include <map>
#include <memory>
#include <random>

#include <boost/shared_ptr.hpp>
#include <gtsam_points/ann/ivox.hpp>
#include <glim/odometry/odometry_estimation_base.hpp>

namespace gtsam {
class Pose3;
class Values;
class ImuFactor;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace gtsam_points {
class IncrementalFixedLagSmootherExt;
class IncrementalFixedLagSmootherExtWithFallback;
}  // namespace gtsam_points

namespace glim {

class IMUIntegration;
class CloudDeskewing;
class CloudCovarianceEstimation;
class InitialStateEstimation;

/**
 * @brief Parameters for OdometryEstimationIMU
 */
struct OdometryEstimationCustomParams {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationCustomParams();
  virtual ~OdometryEstimationCustomParams();

public:
  // Sensor params;
  bool enable_imu;
  bool enable_continuous_time;

  bool fix_imu_bias;
  double imu_bias_noise;
  Eigen::Isometry3d T_lidar_imu;
  Eigen::Matrix<double, 6, 1> imu_bias;

  // Init state
  std::string initialization_mode;
  bool estimate_init_state;
  Eigen::Isometry3d init_T_world_imu;
  Eigen::Vector3d init_v_world_imu;
  double init_pose_damping_scale;

  // Optimization params
  double smoother_lag;
  bool use_isam2_dogleg;
  double isam2_relinearize_skip;
  double isam2_relinearize_thresh;

  double max_correspondence_distance;

  int num_threads;                  // Number of threads for preprocessing and per-factor parallelism
  int num_smoother_update_threads;  // Number of threads for TBB parallelism in smoother update (should be kept 1)
};

/**
 * @brief Base class for LiDAR-IMU odometry estimation
 */
class OdometryEstimationCustom : public OdometryEstimationBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationCustom(std::unique_ptr<OdometryEstimationCustomParams>&& params);
  virtual ~OdometryEstimationCustom() override;

  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;
  virtual EstimationFrame::ConstPtr insert_frame(const PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) override;
  virtual std::vector<EstimationFrame::ConstPtr> get_remaining_frames() override;

protected:
  virtual void fallback_smoother() {}
  virtual gtsam::NonlinearFactorGraph create_factors(const int current, const boost::shared_ptr<gtsam::ImuFactor>& imu_factor, gtsam::Values& new_values);
  virtual void update_frames(const int current, const gtsam::NonlinearFactorGraph& new_factors);

  virtual void
  update_smoother(const gtsam::NonlinearFactorGraph& new_factors, const gtsam::Values& new_values, const std::map<std::uint64_t, double>& new_stamp, int update_count = 0);
  virtual void update_smoother(int update_count = 1);

  void update_target(const int current, const Eigen::Isometry3d& T_target_imu);

protected:
  std::unique_ptr<OdometryEstimationCustomParams> params;

  // Sensor extrinsic params
  Eigen::Isometry3d T_lidar_imu;
  Eigen::Isometry3d T_imu_lidar;

  // Frames & keyframes
  int marginalized_cursor;
  std::vector<EstimationFrame::Ptr> frames;

  // Utility classes
  std::unique_ptr<InitialStateEstimation> init_estimation;
  std::unique_ptr<IMUIntegration> imu_integration;
  std::unique_ptr<CloudDeskewing> deskewing;
  std::unique_ptr<CloudCovarianceEstimation> covariance_estimation;

  // Target model
  std::shared_ptr<gtsam_points::iVox> target_ivox;  ///< GICP target iVox

  // Optimizer
  using FixedLagSmootherExt = gtsam_points::IncrementalFixedLagSmootherExtWithFallback;
  std::unique_ptr<FixedLagSmootherExt> smoother;

  std::shared_ptr<void> tbb_task_arena;
};

}  // namespace glim
