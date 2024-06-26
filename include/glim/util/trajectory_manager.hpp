#pragma once

#include <vector>
#include <iostream>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glim {

class TrajectoryManager {
public:
  TrajectoryManager() {
    odom_stamps.push_back(0.0);
    T_odom_sensor.push_back(Eigen::Isometry3d::Identity());

    T_world_odom.setIdentity();
  }
  ~TrajectoryManager() {}

  void add_odom(double stamp, const Eigen::Isometry3d& T_odom_sensor) {
    this->odom_stamps.push_back(stamp);
    this->T_odom_sensor.push_back(T_odom_sensor);
  }

  void update_anchor(double stamp, const Eigen::Isometry3d& T_world_sensor) {
    auto found = std::lower_bound(odom_stamps.begin(), odom_stamps.end(), stamp);
    int idx = std::distance(odom_stamps.begin(), found);
    T_world_odom = T_world_sensor * T_odom_sensor[idx].inverse();

    if (idx > 1) {
      odom_stamps.erase(odom_stamps.begin(), odom_stamps.begin() + idx - 1);
      T_odom_sensor.erase(T_odom_sensor.begin(), T_odom_sensor.begin() + idx - 1);
    }
  }

  Eigen::Isometry3d current_pose() const { return T_world_odom * T_odom_sensor.back(); }

  Eigen::Isometry3d odom2world(const Eigen::Isometry3d& pose) const { return T_world_odom * pose; }

  Eigen::Vector3d odom2world(const Eigen::Vector3d& point) const { return T_world_odom * point; }

  const Eigen::Isometry3d get_T_world_odom() const { return T_world_odom; }

private:
  std::vector<double> odom_stamps;
  std::vector<Eigen::Isometry3d> T_odom_sensor;

  Eigen::Isometry3d T_world_odom;
};

}  // namespace glim
