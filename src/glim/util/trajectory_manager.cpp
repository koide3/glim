#include <glim/util/trajectory_manager.hpp>

namespace glim {

TrajectoryManager::TrajectoryManager() {
  odom_priority = -1;
  odom_stamps.push_back(0.0);
  T_odom_sensor.push_back(Eigen::Isometry3d::Identity());

  T_world_odom.setIdentity();
}

TrajectoryManager::~TrajectoryManager() {}

void TrajectoryManager::add_odom(double stamp, const Eigen::Isometry3d& T_odom_sensor, int priority) {
  if (odom_priority < priority) {
    odom_priority = priority;
    this->odom_stamps = {0.0};
    this->T_odom_sensor = {T_odom_sensor};
  } else if (odom_priority != priority) {
    return;
  }

  this->odom_stamps.push_back(stamp);
  this->T_odom_sensor.push_back(T_odom_sensor);
}

void TrajectoryManager::update_anchor(double stamp, const Eigen::Isometry3d& T_world_sensor) {
  const auto found = std::lower_bound(odom_stamps.begin(), odom_stamps.end(), stamp);
  const int idx = std::distance(odom_stamps.begin(), found);

  if (std::abs(stamp - odom_stamps[idx]) < 1e-6 || idx == 0) {
    T_world_odom = T_world_sensor * T_odom_sensor[idx].inverse();
  } else {
    const double t0 = odom_stamps[idx - 1];
    const double t1 = odom_stamps[idx];
    if (t0 > stamp || t1 < stamp) {
      // T_world_odom = T_world_sensor * T_odom_sensor[idx].inverse();
      return;
    }

    const double p = (stamp - t0) / (t1 - t0);

    const Eigen::Isometry3d T0 = T_odom_sensor[idx - 1];
    const Eigen::Isometry3d T1 = T_odom_sensor[idx];
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = T0.translation() * (1.0 - p) + T1.translation() * p;
    T.linear() = Eigen::Quaterniond(T0.linear()).slerp(p, Eigen::Quaterniond(T1.linear())).toRotationMatrix();
    T_world_odom = T_world_sensor * T.inverse();
  }

  if (idx > 1) {
    odom_stamps.erase(odom_stamps.begin(), odom_stamps.begin() + idx - 1);
    T_odom_sensor.erase(T_odom_sensor.begin(), T_odom_sensor.begin() + idx - 1);
  }
}

Eigen::Isometry3d TrajectoryManager::current_pose() const {
  return T_world_odom * T_odom_sensor.back();
}

Eigen::Isometry3d TrajectoryManager::odom2world(const Eigen::Isometry3d& pose) const {
  return T_world_odom * pose;
}

Eigen::Vector3d TrajectoryManager::odom2world(const Eigen::Vector3d& point) const {
  return T_world_odom * point;
}

const Eigen::Isometry3d TrajectoryManager::get_T_world_odom() const {
  return T_world_odom;
}

}  // namespace glim