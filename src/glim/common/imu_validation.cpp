#include <glim/common/imu_validation.hpp>

namespace glim {

IMUValidation::IMUValidation(const std::shared_ptr<spdlog::logger>& logger, bool enabled) : enabled(enabled), logger(logger) {
  num_validations = 0;
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

  // Print statistics every 32 validations
  if (num_validations && (num_validations % 32 == 0)) {
    const Eigen::Array3d imu_better_ratios = imu_better_counts.cast<double>() / num_validations;

    bool good_imu = true;

    // Heuristic rules to determine if IMU prediction is good or not.
    // Reference: os1_128_01 dataset, better ratios for rot=1.00, trans=0.55, vel=0.66
    good_imu &= imu_better_ratios[0] > 0.7;  // Rotation
    good_imu &= imu_better_ratios[1] > 0.4;  // Translation
    good_imu &= imu_better_ratios[2] > 0.5;  // Velocity

    auto log_level = (good_imu || num_validations < 100) ? spdlog::level::debug : spdlog::level::warn;

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

}  // namespace glim
