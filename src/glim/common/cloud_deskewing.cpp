#include <glim/common/cloud_deskewing.hpp>

#include <gtsam/geometry/Pose3.h>

namespace glim {

CloudDeskewing::CloudDeskewing() {}

CloudDeskewing::~CloudDeskewing() {}

std::vector<Eigen::Vector4d> CloudDeskewing::deskew(
  const Eigen::Isometry3d& T_imu_lidar,
  const Eigen::Vector3d& linear_vel,
  const Eigen::Vector3d& angular_vel,
  const std::vector<double>& times,
  const std::vector<Eigen::Vector4d>& points) {
  if (times.empty()) {
    return std::vector<Eigen::Vector4d>();
  }

  const Eigen::Isometry3d T_lidar_imu = T_imu_lidar.inverse();
  const gtsam::Vector6 vel = (gtsam::Vector6() << angular_vel, linear_vel).finished();

  const double time_eps = 1e-4;  // 0.1msec
  std::vector<double> time_table;
  std::vector<int> time_indices;

  time_table.reserve((times.back() - times.front()) / time_eps * 1.5);
  time_indices.reserve(points.size());

  for (int i = 0; i < times.size(); i++) {
    if (time_table.empty() || times[i] - time_table.back() > time_eps) {
      time_table.push_back(times[i]);
    }
    time_indices.push_back(time_table.size() - 1);
  }

  std::vector<Eigen::Isometry3d> T_lidar0_lidar1(time_table.size());
  for (int i = 0; i < time_table.size(); i++) {
    const double dt = time_table[i];
    // Maybe this is not correct. Need to check.
    const Eigen::Isometry3d T_imu1_imu0(gtsam::Pose3::Expmap(dt * vel).matrix());
    T_lidar0_lidar1[i] = T_lidar_imu * T_imu1_imu0.inverse() * T_imu_lidar;
  }

  std::vector<Eigen::Vector4d> deskewed(points.size());
  for (int i = 0; i < points.size(); i++) {
    const auto& T_l0_l1 = T_lidar0_lidar1[time_indices[i]];
    deskewed[i] = T_l0_l1 * points[i];
  }

  return deskewed;
}

std::vector<Eigen::Vector4d> CloudDeskewing::deskew(
  const Eigen::Isometry3d& T_imu_lidar,
  const std::vector<double>& imu_times,
  const std::vector<Eigen::Isometry3d>& imu_poses,
  const double stamp,
  const std::vector<double>& times,
  const std::vector<Eigen::Vector4d>& points) {
  //
  if (times.empty()) {
    return std::vector<Eigen::Vector4d>();
  }

  if (imu_poses.empty()) {
    return deskew(T_imu_lidar, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), times, points);
  }

  const double time_eps = 1e-4;
  std::vector<double> time_table;
  std::vector<int> time_indices;
  time_table.reserve((times.back() - times.front()) / time_eps * 1.2);
  time_indices.reserve(times.size());

  // create time table
  for (const double t : times) {
    if (time_table.empty() || t - time_table.back() > time_eps) {
      time_table.push_back(t);
    }
    time_indices.push_back(time_table.size() - 1);
  }

  const Eigen::Isometry3d T_lidar_imu = T_imu_lidar.inverse();
  std::vector<Eigen::Isometry3d> T_lidar0_lidar1(time_table.size());

  int imu_cursor = 0;
  Eigen::Isometry3d T_imu0_world;  // IMU pose at the very beginning of the scan

  // Calculate T_lidar0_lidar1 for each time in time table
  for (int i = 0; i < time_table.size(); i++) {
    const double time = stamp + time_table[i];

    while (imu_cursor < imu_times.size() - 1 && imu_times[imu_cursor + 1] < time) {
      imu_cursor++;
    }

    if (i == 0) {
      // Should interpolate?
      T_imu0_world = imu_poses[imu_cursor].inverse();
    }

    Eigen::Isometry3d T_world_imu1 = Eigen::Isometry3d::Identity();
    if (imu_cursor + 1 >= imu_times.size()) {
      T_world_imu1 = imu_poses[imu_cursor];
    } else {
      const double imu_t0 = imu_times[imu_cursor];
      const double imu_t1 = imu_times[imu_cursor + 1];

      const double p = std::max(0.0, std::min(1.0, (time - imu_t0) / (imu_t1 - imu_t0)));

      const Eigen::Vector3d imu_trans_l = imu_poses[imu_cursor].translation();
      const Eigen::Vector3d imu_trans_r = imu_poses[imu_cursor + 1].translation();
      const Eigen::Quaterniond imu_quat_l(imu_poses[imu_cursor].linear());
      const Eigen::Quaterniond imu_quat_r(imu_poses[imu_cursor + 1].linear());

      T_world_imu1.translation() = (1.0 - p) * imu_trans_l + p * imu_trans_r;
      T_world_imu1.linear() = imu_quat_l.slerp(p, imu_quat_r).toRotationMatrix();
    }

    const Eigen::Isometry3d T_imu0_imu1 = T_imu0_world * T_world_imu1;
    T_lidar0_lidar1[i] = T_lidar_imu * T_imu0_imu1 * T_imu_lidar;
  }

  // Transform points
  std::vector<Eigen::Vector4d> deskewed(points.size());
  for (int i = 0; i < points.size(); i++) {
    deskewed[i] = T_lidar0_lidar1[time_indices[i]] * points[i];
  }

  return deskewed;
}

}  // namespace glim