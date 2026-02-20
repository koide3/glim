#include <glim/odometry/initial_state_estimation.hpp>

#include <iostream>
#include <spdlog/spdlog.h>
#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>

namespace glim {

InitialStateEstimation::InitialStateEstimation() : logger(create_module_logger("odom")) {}

NaiveInitialStateEstimation::NaiveInitialStateEstimation(const Eigen::Isometry3d& T_base_imu, const Eigen::Isometry3d& T_base_lidar, const Eigen::Matrix<double, 6, 1>& imu_bias)
: ready(false),
  init_stamp(0.0),
  stamp(0.0),
  sum_acc(Eigen::Vector3d::Zero()),
  imu_bias(imu_bias),
  T_base_imu(T_base_imu),
  T_base_lidar(T_base_lidar),
  force_init(false),
  init_T_world_base(Eigen::Isometry3d::Identity()),
  init_v_world_base(Eigen::Vector3d::Zero()) {
  glim::Config config(glim::GlobalConfig::get_config_path("config_odometry"));
  window_size = config.param("odometry_estimation", "initialization_window_size", 1.0);
}

NaiveInitialStateEstimation::~NaiveInitialStateEstimation() {}

void NaiveInitialStateEstimation ::set_init_state(const Eigen::Isometry3d& init_T_world_base, const Eigen::Vector3d& init_v_world_base) {
  force_init = true;
  this->init_T_world_base = init_T_world_base;
  this->init_v_world_base = init_v_world_base;

  this->init_T_world_base.linear() = Eigen::Quaterniond(this->init_T_world_base.linear()).normalized().toRotationMatrix();
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

  this->stamp = stamp;
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

  estimated->T_base_imu = T_base_imu;
  estimated->T_base_lidar = T_base_lidar;
  estimated->imu_bias = imu_bias;

  estimated->v_world_imu.setZero();

  if (force_init) {
    estimated->v_world_imu = init_v_world_base;
    estimated->set_T_world_sensor(FrameID::BASE, init_T_world_base);
  } else {
    Eigen::Isometry3d T_world_imu = Eigen::Isometry3d::Identity();

    Eigen::Vector3d acc_dir = sum_acc.normalized();

    if (acc_dir.dot(Eigen::Vector3d::UnitZ()) < 0.999) {
      const Eigen::Vector3d axis = Eigen::Vector3d::UnitZ().cross(acc_dir).normalized();
      const double angle = std::acos(acc_dir.dot(Eigen::Vector3d::UnitZ()));

      T_world_imu.linear() = Eigen::AngleAxisd(-angle, axis).toRotationMatrix();
    }

    estimated->v_world_imu.setZero();
    estimated->set_T_world_sensor(FrameID::IMU, T_world_imu);
  }

  logger->info("initial IMU state estimation done");
  logger->info("Init Base Pose: Trans=[{}, {}, {}]", 
        estimated->T_world_base.translation().x(),
        estimated->T_world_base.translation().y(),
        estimated->T_world_base.translation().z());
        
  return estimated;
}

}  // namespace glim