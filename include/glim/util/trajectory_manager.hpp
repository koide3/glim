#pragma once

#include <vector>
#include <iostream>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glim {

class TrajectoryManager {
public:
  TrajectoryManager();
  ~TrajectoryManager();

  void add_odom(double stamp, const Eigen::Isometry3d& T_odom_sensor, int priority = 1);
  void update_anchor(double stamp, const Eigen::Isometry3d& T_world_sensor);

  Eigen::Isometry3d current_pose() const;
  Eigen::Isometry3d odom2world(const Eigen::Isometry3d& pose) const;
  Eigen::Vector3d odom2world(const Eigen::Vector3d& point) const;
  const Eigen::Isometry3d get_T_world_odom() const;

private:
  int odom_priority;
  std::vector<double> odom_stamps;
  std::vector<Eigen::Isometry3d> T_odom_sensor;

  Eigen::Isometry3d T_world_odom;
};

}  // namespace glim
