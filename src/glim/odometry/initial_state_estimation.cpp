#include <glim/odometry/initial_state_estimation.hpp>

#include <iostream>
#include <spdlog/spdlog.h>
#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>

namespace glim {

InitialStateEstimation::InitialStateEstimation() : logger(create_module_logger("odom")) {}

NaiveInitialStateEstimation::NaiveInitialStateEstimation(const Eigen::Isometry3d& T_lidar_imu, const Eigen::Matrix<double, 6, 1>& imu_bias)
: ready(false),
  init_stamp(0.0),
  stamp(0.0),
  sum_acc(Eigen::Vector3d::Zero()),
  imu_bias(imu_bias),
  T_lidar_imu(T_lidar_imu),
  force_init(false),
  init_T_world_imu(Eigen::Isometry3d::Identity()),
  init_v_world_imu(Eigen::Vector3d::Zero()) {
  glim::Config config(glim::GlobalConfig::get_config_path("config_odometry"));
  window_size = config.param("odometry_estimation", "initialization_window_size", 1.0);
}

NaiveInitialStateEstimation::~NaiveInitialStateEstimation() {}

void NaiveInitialStateEstimation ::set_init_state(const Eigen::Isometry3d& init_T_world_imu, const Eigen::Vector3d& init_v_world_imu) {
  force_init = true;
  this->init_T_world_imu = init_T_world_imu;
  this->init_v_world_imu = init_v_world_imu;

  this->init_T_world_imu.linear() = Eigen::Quaterniond(this->init_T_world_imu.linear()).normalized().toRotationMatrix();
}

void NaiveInitialStateEstimation::insert_imu(double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  if (linear_acc.norm() < 5.0 || linear_acc.norm() > 15.0) {
    logger->warn("too large or small acc found ({}[m/s^2])", linear_acc.norm());
  }

  if (init_stamp < 1e-6) {
    init_stamp = stamp;
  }

  if (ready) {
    return;
  }

  stamp = stamp;
  sum_acc += linear_acc;

  ready = stamp - init_stamp > window_size && sum_acc.squaredNorm() > 10.0;
}

EstimationFrame::ConstPtr NaiveInitialStateEstimation::initial_pose() {
  if (!force_init && !ready) {
    return nullptr;
  }

  EstimationFrame::Ptr estimated(new EstimationFrame);
  estimated->id = -1;
  estimated->stamp = stamp;
  estimated->T_lidar_imu = T_lidar_imu;
  estimated->v_world_imu.setZero();
  estimated->imu_bias = imu_bias;

  if (force_init) {
    estimated->v_world_imu = init_v_world_imu;
    estimated->T_world_imu = init_T_world_imu;
  } else {
    Eigen::Vector3d acc_dir = sum_acc.normalized();
    if (acc_dir.dot(Eigen::Vector3d::UnitZ()) < 0.999) {
      const Eigen::Vector3d axis = Eigen::Vector3d::UnitZ().cross(acc_dir).normalized();
      const double angle = std::acos(acc_dir.dot(Eigen::Vector3d::UnitZ()));

      estimated->T_world_imu.setIdentity();
      estimated->T_world_imu.linear() = Eigen::AngleAxisd(-angle, axis).toRotationMatrix();
    } else {
      estimated->T_world_imu = Eigen::Isometry3d::Identity();
    }
  }

  estimated->T_world_lidar = estimated->T_world_imu * T_lidar_imu.inverse();

  logger->info("initial IMU state estimation done");

  return estimated;
}

}  // namespace glim