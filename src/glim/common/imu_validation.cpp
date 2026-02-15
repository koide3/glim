#include <glim/common/imu_validation.hpp>

namespace glim {

IMUValidation::IMUValidation(const std::shared_ptr<spdlog::logger>& logger, bool enabled) : enabled(enabled), logger(logger) {
  num_validations = 0;
  num_bias_validations = 0;
  imu_better_counts.setZero();
}

IMUValidation::~IMUValidation() {}

void IMUValidation::validate(
  const Eigen::Isometry3d& last_T_world_imu,
  const Eigen::Vector3d& last_v_world_imu,
  const Eigen::Isometry3d& predicted_T_world_imu,
  const Eigen::Vector3d& predicted_v_world_imu,
  const Eigen::Isometry3d& corrected_T_world_imu,
  const Eigen::Vector3d& corrected_v_world_imu,
  double dt) {
  if (!enabled) {
    return;
  }

  logger->debug("validating IMU prediction (|v|={:.3f} m/s)", corrected_v_world_imu.norm());
  if (corrected_v_world_imu.norm() < 0.1) {
    // Skip validation when the velocity is too small, as the errors can be dominated by noise.
    logger->debug("skipping IMU validation due to small velocity");
    return;
  }

  Eigen::Isometry3d noimu_T_world_imu = last_T_world_imu;
  noimu_T_world_imu.translation() += last_v_world_imu * dt;
  const auto& noimu_v_world_imu = last_v_world_imu;

  const Eigen::Isometry3d noimu_error_T = corrected_T_world_imu.inverse() * noimu_T_world_imu;
  const Eigen::Vector3d noimu_error_v = corrected_v_world_imu - noimu_v_world_imu;

  const Eigen::Isometry3d imu_error_T = corrected_T_world_imu.inverse() * predicted_T_world_imu;
  const Eigen::Vector3d imu_error_v = corrected_v_world_imu - predicted_v_world_imu;

  const double noimu_error_rot = Eigen::AngleAxisd(noimu_error_T.linear()).angle();
  const double noimu_error_trans = noimu_error_T.translation().norm();
  const double noimu_error_vel = noimu_error_v.norm();
  const Eigen::Array3d noimu_errors(noimu_error_rot, noimu_error_trans, noimu_error_vel);
  noimu_error_stats.add(noimu_errors);

  const double imu_error_rot = Eigen::AngleAxisd(imu_error_T.linear()).angle();
  const double imu_error_trans = imu_error_T.translation().norm();
  const double imu_error_vel = imu_error_v.norm();
  const Eigen::Array3d imu_errors(imu_error_rot, imu_error_trans, imu_error_vel);
  imu_error_stats.add(imu_errors);

  imu_better_counts += (imu_errors < noimu_errors).cast<int>();
  num_validations++;

  // Print statistics every 64 validations
  if (num_validations % 64 == 0) {
    const Eigen::Array3d imu_better_ratios = imu_better_counts.cast<double>() / num_validations;

    bool good_imu = true;

    // Heuristic rules to determine if IMU prediction is good or not.
    // Reference: os1_128_01 dataset, better ratios for rot=1.00, trans=0.55, vel=0.66
    good_imu &= imu_better_ratios[0] > 0.7;  // Rotation
    good_imu &= imu_better_ratios[1] > 0.4;  // Translation
    good_imu &= imu_better_ratios[2] > 0.5;  // Velocity

    auto log_level = good_imu ? spdlog::level::debug : spdlog::level::warn;

    const Eigen::Array3d noimu_error_means = noimu_error_stats.mean();
    const Eigen::Array3d noimu_error_stds = noimu_error_stats.std();
    const Eigen::Array3d imu_error_means = imu_error_stats.mean();
    const Eigen::Array3d imu_error_stds = imu_error_stats.std();

    if (!good_imu) {
      logger->log(log_level, "IMU prediction is not good.");
      logger->log(log_level, "Possibly T_lidar_imu is not accurate or IMU bias is not well estimated.");
    }
    logger->log(log_level, "IMU validation results:");
    logger->log(log_level, "num_validations={}", num_validations);
    logger->log(
      log_level,
      "No-IMU errors rot={:.3f} +- {:.3f} deg, trans={:.3f} +- {:.3f} m, vel={:.3f} +- {:.3f} m/s",
      noimu_error_means[0] * 180.0 / M_PI,
      noimu_error_stds[0] * 180.0 / M_PI,
      noimu_error_means[1],
      noimu_error_stds[1],
      noimu_error_means[2],
      noimu_error_stds[2]);
    logger->log(
      log_level,
      "IMU errors rot={:.3f} +- {:.3f} deg, trans={:.3f} +- {:.3f} m, vel={:.3f} +- {:.3f} m/s",
      imu_error_means[0] * 180.0 / M_PI,
      imu_error_stds[0] * 180.0 / M_PI,
      imu_error_means[1],
      imu_error_stds[1],
      imu_error_means[2],
      imu_error_stds[2]);
    logger->log(log_level, "IMU better ratios rot={:.2f}, trans={:.2f}, vel={:.2f}", imu_better_ratios[0], imu_better_ratios[1], imu_better_ratios[2]);
  }
}

void IMUValidation::validate(const Eigen::Matrix<double, 6, 1>& imu_bias) {
  if (!enabled) {
    return;
  }

  num_bias_validations++;
  if (num_bias_validations % 64) {
    return;
  }

  const Eigen::Array3d bias_acc = imu_bias.head<3>();
  const Eigen::Array3d bias_gyro = imu_bias.tail<3>();

  bool good_imu = true;
  good_imu &= bias_acc.cwiseAbs().maxCoeff() < 0.5;   // Acceleration bias should be small (less than 0.5 m/s^2)
  good_imu &= bias_gyro.cwiseAbs().maxCoeff() < 0.5;  // Gyro bias should be small (less than 0.5 rad/s)

  imu_bias_stats.add(imu_bias);
  const Eigen::Array<double, 6, 1> bias_mean = imu_bias_stats.mean();
  const Eigen::Array<double, 6, 1> bias_std = imu_bias_stats.std();
  const Eigen::Array<double, 6, 1> bias_max = imu_bias_stats.max();
  const Eigen::Array<double, 6, 1> bias_min = imu_bias_stats.min();

  auto log_level = good_imu ? spdlog::level::debug : spdlog::level::warn;
  if (!good_imu) {
    logger->log(log_level, "IMU bias estimation seems inaccurate.");
    logger->log(log_level, "Possibly T_lidar_imu is not accurate or LiDAR-IMU synchronization is not correct.");
  }
  logger->log(log_level, "IMU bias validation results:");
  logger->log(log_level, "num_bias_validations={}", num_bias_validations);
  logger->log(
    log_level,
    "bias_acc=[{:.3f}, {:.3f}, {:.3f}] m/s^2, bias_gyro=[{:.3f}, {:.3f}, {:.3f}] rad/s",
    bias_acc[0],
    bias_acc[1],
    bias_acc[2],
    bias_gyro[0],
    bias_gyro[1],
    bias_gyro[2]);
  logger->log(
    log_level,
    "acc_stat ax={:.3f} +- {:.3f} [min={:.3f}, max={:.3f}], ay={:.3f} +- {:.3f} [min={:.3f}, max={:.3f}], az={:.3f} +- {:.3f} [min={:.3f}, max={:.3f}] m/s^2",
    bias_mean[0],
    bias_std[0],
    bias_min[0],
    bias_max[0],
    bias_mean[1],
    bias_std[1],
    bias_min[1],
    bias_max[1],
    bias_mean[2],
    bias_std[2],
    bias_min[2],
    bias_max[2]);
  logger->log(
    log_level,
    "gyro_stat wx={:.3f} +- {:.3f} [min={:.3f}, max={:.3f}], wy={:.3f} +- {:.3f} [min={:.3f}, max={:.3f}], wz={:.3f} +- {:.3f} [min={:.3f}, max={:.3f}] rad/s",
    bias_mean[3],
    bias_std[3],
    bias_min[3],
    bias_max[3],
    bias_mean[4],
    bias_std[4],
    bias_min[4],
    bias_max[4],
    bias_mean[5],
    bias_std[5],
    bias_min[5],
    bias_max[5]);
}

}  // namespace glim
