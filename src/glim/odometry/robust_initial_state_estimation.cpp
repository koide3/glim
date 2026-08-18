#include <glim/odometry/robust_initial_state_estimation.hpp>

#include <sstream>
#include <spdlog/spdlog.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim/util/config.hpp>
#include <glim/util/convert_to_string.hpp>
#include <glim/common/imu_integration.hpp>
#include <glim/common/cloud_deskewing.hpp>
#include <glim/common/cloud_covariance_estimation.hpp>
#include <glim/odometry/callbacks.hpp>

namespace glim {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::G;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

RobustInitialStateEstimation::RobustInitialStateEstimation(const Eigen::Isometry3d& T_lidar_imu) : T_lidar_imu(T_lidar_imu), use_nonlinear_refinement(false) {
  glim::Config config(glim::GlobalConfig::get_config_path("config_odometry"));
  num_threads = config.param("odometry_estimation", "num_threads", 2);
  window_size = config.param("odometry_estimation", "initialization_window_size", 1.0);

  target_ivox.reset(new gtsam_points::iVox(1.0));
  deskewing.reset(new CloudDeskewing());
  covariance_estimation.reset(new CloudCovarianceEstimation(num_threads));
  imu_integration.reset(new glim::IMUIntegration());
}

RobustInitialStateEstimation::~RobustInitialStateEstimation() {}

void RobustInitialStateEstimation::enable_nonlinear_refinement() {
  use_nonlinear_refinement = true;
}

void RobustInitialStateEstimation::insert_imu(double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  imu_integration->insert_imu(stamp, linear_acc, angular_vel);
}

void RobustInitialStateEstimation::insert_frame(const PreprocessedFrame::ConstPtr& raw_frame) {
  if (raw_frame->size() < 50) {
    logger->warn("skip initial state estimation for a frame with too few points ({} points)", raw_frame->size());
    return;
  }

  // Update odometry
  update_odom(raw_frame);

  logger->debug("inserted a frame for initial state estimation: stamp={} |Ts_odom_lidar|={}", raw_frame->stamp, Ts_odom_lidar.size());
  IMUStateInitializationCallbacks::on_updated(raw_frame, Ts_odom_lidar.back().second);
}

EstimationFrame::ConstPtr RobustInitialStateEstimation::initial_pose() {
  logger->debug("initial_pose() called: |Ts_odom_lidar|={}", Ts_odom_lidar.size());
  if (Ts_odom_lidar.empty() || Ts_odom_lidar.back().first - Ts_odom_lidar.front().first < window_size) {
    logger->debug(
      "not enough data for initial state estimation: |Ts_odom_lidar|={} time_range={}",
      Ts_odom_lidar.size(),
      Ts_odom_lidar.empty() ? 0.0 : (Ts_odom_lidar.back().first - Ts_odom_lidar.front().first));
    return nullptr;
  }

  if (imu_integration->imu_data_in_queue().empty()) {
    logger->warn("no IMU data for initial state estimation");
    return nullptr;
  }

  logger->debug("initial estimation");
  Eigen::Vector3d gyro_bias = estimate_gyro_bias();
  gtsam::Values init_values = estimate_linear_system(gyro_bias);
  if (init_values.empty()) {
    logger->warn("Failed to estimate linear system for initial state estimation");
    return nullptr;
  }

  // Align the gravity direction of the odom frame with the world downward direction (0, 0, -1)
  logger->debug("aligning odom frame with world frame");
  const Eigen::Vector3d world_gravity(0.0, 0.0, -1.0);
  const Eigen::Vector3d odom_gravity = init_values.at<gtsam::Vector3>(G(0)).normalized();

  Eigen::Isometry3d T_world_odom = Eigen::Isometry3d::Identity();
  T_world_odom.linear() = Eigen::Quaterniond::FromTwoVectors(odom_gravity, world_gravity).matrix();

  const Eigen::Vector3d acc_bias = init_values.at<gtsam::Vector3>(B(0));
  const gtsam::imuBias::ConstantBias imu_bias(acc_bias, gyro_bias);
  gtsam::Values values;
  values.insert(B(0), imu_bias);

  for (int i = 0; i < Ts_odom_lidar.size(); i++) {
    const Eigen::Isometry3d T_odom_imu = Ts_odom_lidar[i].second * T_lidar_imu;
    const Eigen::Isometry3d T_world_imu = T_world_odom * T_odom_imu;
    const Eigen::Vector3d v_odom_imu = init_values.at<gtsam::Vector3>(V(i));
    const Eigen::Vector3d v_world_imu = T_world_odom.linear() * v_odom_imu;

    values.insert(X(i), gtsam::Pose3(T_world_imu.matrix()));
    values.insert(V(i), v_world_imu);
  }

  // Refine the initial estimation using nonlinear optimization
  if (use_nonlinear_refinement) {
    logger->debug("refining initial estimation using nonlinear optimization");
    values = refine_nonlinear(values);
  }

  // Retrieve the estimated state of the latest frame
  logger->debug("retrieving the estimated state of the latest frame");
  const int last_idx = Ts_odom_lidar.size() - 1;
  EstimationFrame::Ptr estimated(new EstimationFrame);
  estimated->id = -1;
  estimated->stamp = Ts_odom_lidar[last_idx].first;
  estimated->T_lidar_imu = T_lidar_imu;
  estimated->v_world_imu = values.at<gtsam::Vector3>(V(last_idx));
  estimated->imu_bias = values.at<gtsam::imuBias::ConstantBias>(B(0)).vector();

  estimated->T_world_imu = Eigen::Isometry3d(values.at<gtsam::Pose3>(X(last_idx)).matrix());
  estimated->T_world_lidar = estimated->T_world_imu * T_lidar_imu.inverse();

  IMUStateInitializationCallbacks::on_finished(estimated);

  return estimated;
}

}  // namespace glim
