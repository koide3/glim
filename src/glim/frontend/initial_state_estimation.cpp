#include <glim/frontend/initial_state_estimation.hpp>

#include <iostream>
#include <glim/util/config.hpp>

namespace glim {

NaiveInitialStateEstimation::NaiveInitialStateEstimation() : stamp(0.0), sum_acc(Eigen::Vector3d::Zero()) {
  Config config(GlobalConfig::get_config_path("config_sensors"));
  T_lidar_imu = config.param<Eigen::Isometry3d>("sensors", "T_lidar_imu", Eigen::Isometry3d::Identity());

  auto bias = config.param<std::vector<double>>("sensors", "imu_bias");
  if (bias && bias->size() == 6) {
    imu_bias = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(bias->data());
  } else {
    imu_bias.setZero();
  }
}

NaiveInitialStateEstimation ::~NaiveInitialStateEstimation() {}

void NaiveInitialStateEstimation::insert_imu(double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  stamp = stamp;
  sum_acc += linear_acc;
}

EstimationFrame::ConstPtr NaiveInitialStateEstimation::initial_pose() {
  if (sum_acc.squaredNorm() < 1.0) {
    return nullptr;
  }

  EstimationFrame::Ptr estimated(new EstimationFrame);
  estimated->id = -1;
  estimated->stamp = stamp;
  estimated->T_lidar_imu = T_lidar_imu;
  estimated->v_world_imu.setZero();
  estimated->imu_bias = imu_bias;

  Eigen::Vector3d acc_dir = sum_acc.normalized();
  if (acc_dir.dot(Eigen::Vector3d::UnitZ()) < 0.999) {
    const Eigen::Vector3d axis = Eigen::Vector3d::UnitZ().cross(acc_dir).normalized();
    const double angle = std::acos(acc_dir.dot(Eigen::Vector3d::UnitZ()));

    estimated->T_world_imu.setIdentity();
    estimated->T_world_imu.linear() = Eigen::AngleAxisd(-angle, axis).toRotationMatrix();
  } else {
    estimated->T_world_imu = Eigen::Isometry3d::Identity();
  }

  estimated->T_world_lidar = estimated->T_world_imu * T_lidar_imu.inverse();

  return estimated;
}

}  // namespace glim