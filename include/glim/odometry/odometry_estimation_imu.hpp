#pragma once

#include <map>
#include <deque>
#include <memory>
#include <random>

#include <GeographicLib/LocalCartesian.hpp>

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
struct OdometryEstimationIMUParams {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationIMUParams();
  virtual ~OdometryEstimationIMUParams();

public:
  // Sensor params;
  bool fix_imu_bias;
  double imu_bias_noise;
  Eigen::Matrix<double, 6, 1> imu_bias;

  Eigen::Isometry3d T_base_imu;
  Eigen::Isometry3d T_base_gnss;
  Eigen::Isometry3d T_base_lidar;
  Eigen::Isometry3d T_odom_gnss_world;

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

  // GNSS fusion params
  double gps_noise_std;            // Standard deviation for GPS position noise (m)
  double gps_velocity_noise_std;   // Standard deviation for GPS velocity noise (m/s)
  double rtk_loss_position_scale;  // Scale factor for position noise when RTK lost
  double gps_z_sigma_min;          // Minimum Z sigma for GPS altitude constraint (m), prevents over-constraining

  // Logging params
  bool save_imu_rate_trajectory;

  int num_threads;                  // Number of threads for preprocessing and per-factor parallelism
  int num_smoother_update_threads;  // Number of threads for TBB parallelism in smoother update (should be kept 1)
};

/**
 * @brief Base class for LiDAR-IMU odometry estimation
 */
class OdometryEstimationIMU : public OdometryEstimationBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationIMU(std::unique_ptr<OdometryEstimationIMUParams>&& params);
  virtual ~OdometryEstimationIMU() override;

  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;
  virtual void insert_gnss(const double stamp, const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Vector3d& var, bool is_rtk_fixed = true) override;
  virtual EstimationFrame::ConstPtr insert_frame(const PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) override;
  virtual std::vector<EstimationFrame::ConstPtr> get_remaining_frames() override;

protected:
  virtual void create_frame(EstimationFrame::Ptr& frame) {}
  virtual gtsam::NonlinearFactorGraph create_factors(const int current, const std::shared_ptr<gtsam::ImuFactor>& imu_factor, gtsam::Values& new_values) = 0;
  virtual gtsam::NonlinearFactorGraph create_gnss_factor(
    const int current,
    gtsam::Values& new_values,
    const std::vector<double>& imu_pred_times,
    const std::vector<Eigen::Isometry3d>& imu_pred_poses);

  virtual void fallback_smoother() {}
  virtual void update_frames(const int current, const gtsam::NonlinearFactorGraph& new_factors);

  virtual void
  update_smoother(const gtsam::NonlinearFactorGraph& new_factors, const gtsam::Values& new_values, const std::map<std::uint64_t, double>& new_stamp, int update_count = 0);
  virtual void update_smoother(int update_count = 1);

protected:
  std::unique_ptr<OdometryEstimationIMUParams> params;

  // GNSS measurement buffer
  struct GNSSMeasurement {
    double stamp;
    Eigen::Vector3d position;  // Position in odometry frame
    Eigen::Vector3d velocity;  // Velocity in odometry frame (Doppler-derived)
    Eigen::Vector3d variance;
    bool is_rtk_fixed;  // True if RTK Fix, False if Float/Single
  };
  std::deque<GNSSMeasurement> gnss_buffer;

  // Geographic converter (for WGS84 to local conversion if needed)
  GeographicLib::LocalCartesian geo_converter;
  bool gnss_set_origin{false};

  // Sensor extrinsic params
  Eigen::Isometry3d T_base_imu;
  Eigen::Isometry3d T_base_gnss;
  Eigen::Isometry3d T_base_lidar;

  Eigen::Isometry3d T_lidar_imu;
  Eigen::Isometry3d T_imu_lidar;
  Eigen::Vector3d t_base_gnss;

  // ENU→World calibration (FLU world frame vs ENU GNSS frame).
  // GLIM world = FLU (X=Forward, Y=Left, Z=Up).
  // GNSS output = ENU (X=East, Y=North, Z=Up).
  // Calibrated once via velocity matching when speed > 1.5 m/s:
  //   R_world_enu = Rz(-enu_yaw_world_x_)   [yaw of vehicle heading in ENU]
  //   t_world_enu_                            [ENU origin in world frame]
  // Before calibration, GNSS factors are skipped.
  bool enu_calibrated_{false};
  double enu_yaw_world_x_{0.0};          // ENU heading of world-X axis (rad, CCW from East)
  Eigen::Vector3d t_world_enu_{0, 0, 0}; // ENU origin expressed in world frame

  // Frames & keyframes
  int marginalized_cursor;
  std::vector<EstimationFrame::Ptr> frames;
  uint64_t gnss_id{0};

  // Utility classes
  std::unique_ptr<InitialStateEstimation> init_estimation;
  std::unique_ptr<IMUIntegration> imu_integration;
  std::unique_ptr<CloudDeskewing> deskewing;
  std::unique_ptr<CloudCovarianceEstimation> covariance_estimation;

  // Optimizer
  using FixedLagSmootherExt = gtsam_points::IncrementalFixedLagSmootherExtWithFallback;
  std::unique_ptr<FixedLagSmootherExt> smoother;

  std::shared_ptr<void> tbb_task_arena;
};

}  // namespace glim
