#include <glim/util/trajectory_manager.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace {
Eigen::Isometry3d odom_pose(double stamp) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(stamp, 2.0 * stamp, -stamp);
  pose.linear() = Eigen::AngleAxisd(0.2 * stamp, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

Eigen::Isometry3d world_offset() {
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(10.0, -4.0, 2.0);
  offset.linear() = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitX()).toRotationMatrix();
  return offset;
}

bool equal_pose(const Eigen::Isometry3d& actual, const Eigen::Isometry3d& expected) {
  return actual.matrix().allFinite() && (actual.matrix() - expected.matrix()).norm() < 1e-10;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) return 1;
  const std::string name = argv[1];
  glim::TrajectoryManager manager;
  const auto offset = world_offset();
  const auto unused_anchor = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d expected = offset;
  Eigen::Isometry3d latest = odom_pose(3.0);

  if (name == "before_first_odom") {
    manager.update_anchor(1.0, offset);
    expected.setIdentity();
    latest.setIdentity();
  } else {
    manager.add_odom(1.0, odom_pose(1.0));
    manager.add_odom(3.0, latest);
    manager.update_anchor(2.0, offset * odom_pose(2.0));

    if (name == "future_anchor") {
      manager.update_anchor(4.0, unused_anchor);
    } else if (name == "just_after_latest") {
      manager.update_anchor(std::nextafter(3.0, std::numeric_limits<double>::infinity()), unused_anchor);
    } else if (name == "repeated_future_anchors") {
      for (double stamp : {4.0, 5.0, 100.0}) manager.update_anchor(stamp, unused_anchor);
    } else if (name == "retry_after_odom") {
      manager.update_anchor(4.0, unused_anchor);
      latest = odom_pose(5.0);
      manager.add_odom(5.0, latest);
      expected = offset.inverse();
      manager.update_anchor(4.0, expected * odom_pose(4.0));
    } else if (name == "exact_latest") {
      expected = offset.inverse();
      manager.update_anchor(3.0, expected * latest);
    } else if (name == "after_pruning") {
      manager.update_anchor(3.0, offset * latest);
      latest = odom_pose(5.0);
      manager.add_odom(5.0, latest);
      manager.update_anchor(6.0, unused_anchor);
      // The earlier retained bracket must still support an in-range update.
      expected = offset.inverse();
      manager.update_anchor(2.0, expected * odom_pose(2.0));
    } else if (name != "interpolation") {
      std::cerr << "Unknown test: " << name << '\n';
      return 1;
    }
  }

  const Eigen::Vector3d point(1.0, -2.0, 3.0);
  if (
    !equal_pose(manager.get_T_world_odom(), expected) || !equal_pose(manager.current_pose(), expected * latest) || !equal_pose(manager.odom2world(latest), expected * latest) ||
    (manager.odom2world(point) - expected * point).norm() > 1e-10) {
    std::cerr << name << ": trajectory transform changed unexpectedly\n";
    return 1;
  }
  return 0;
}
