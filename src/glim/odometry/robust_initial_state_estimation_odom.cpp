#include <glim/odometry/robust_initial_state_estimation.hpp>

#include <sstream>
#include <spdlog/spdlog.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>

#include <glim/util/config.hpp>
#include <glim/util/convert_to_string.hpp>
#include <glim/common/imu_integration.hpp>
#include <glim/common/cloud_deskewing.hpp>
#include <glim/common/cloud_covariance_estimation.hpp>
#include <glim/odometry/callbacks.hpp>

namespace glim {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::G;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

void RobustInitialStateEstimation::update_odom(const PreprocessedFrame::ConstPtr& raw_frame) {
  logger->debug("update init_est odometry for frame at time {} with {} points", raw_frame->stamp, raw_frame->size());

  // Deskew the point cloud using IMU pose prediction
  std::vector<double> imu_pred_times;
  std::vector<Eigen::Isometry3d> imu_pred_poses;
  imu_integration->integrate_imu(raw_frame->stamp, raw_frame->scan_end_time, gtsam::NavState(), gtsam::imuBias::ConstantBias(), imu_pred_times, imu_pred_poses);

  for (auto& pred_pose : imu_pred_poses) {
    // Use only rotation for deskewing
    pred_pose.translation().setZero();
  }

  auto deskewed_points = deskewing->deskew(T_lidar_imu.inverse(), imu_pred_times, imu_pred_poses, raw_frame->stamp, raw_frame->times, raw_frame->points);
  auto frame = std::make_shared<gtsam_points::PointCloudCPU>(deskewed_points);
  frame->add_covs(covariance_estimation->estimate(deskewed_points, raw_frame->neighbors));

  // Initial pose prediction
  gtsam::Pose3 T_odom_lidar = gtsam::Pose3::Identity();

  if (Ts_odom_lidar.size() >= 1) {
    // Use the last odometry pose as the initial guess
    T_odom_lidar = gtsam::Pose3(Ts_odom_lidar.back().second.matrix());

    // Rotation prediction using IMU integration
    const auto& prev = Ts_odom_lidar.back();

    int num_integrated = 0;
    imu_integration->integrate_imu(prev.first, raw_frame->stamp, gtsam::imuBias::ConstantBias(), &num_integrated);

    // The preintegrated delta is defined in the IMU frame
    const Eigen::Isometry3d prev_T_odom_imu = prev.second * T_lidar_imu;
    gtsam::NavState prev_state(gtsam::Pose3(prev_T_odom_imu.matrix()), gtsam::Vector3::Zero());
    const auto pred_T_odom_imu = imu_integration->integrated_measurements().predict(prev_state, gtsam::imuBias::ConstantBias());
    const gtsam::Rot3 pred_R_odom_lidar = pred_T_odom_imu.rotation() * gtsam::Rot3(T_lidar_imu.linear()).inverse();

    // Apply only rotation prediction to the initial pose
    T_odom_lidar = gtsam::Pose3(pred_R_odom_lidar, T_odom_lidar.translation());
  }

  // Linear motion prediction using odometry
  if (Ts_odom_lidar.size() >= 2) {
    const auto& odom0 = Ts_odom_lidar[Ts_odom_lidar.size() - 2];
    const auto& odom1 = Ts_odom_lidar[Ts_odom_lidar.size() - 1];
    const double dt = odom1.first - odom0.first;

    if (dt > 1e-3) {
      const Eigen::Vector3d v = (odom1.second.translation() - odom0.second.translation()) / dt;
      const Eigen::Vector3d delta = v * (raw_frame->stamp - odom1.first);
      T_odom_lidar = gtsam::Pose3(T_odom_lidar.rotation(), T_odom_lidar.translation() + delta);
    }
  }

  if (!Ts_odom_lidar.empty()) {
    // Perform point cloud registration
    gtsam::Values values;
    values.insert(0, T_odom_lidar);

    gtsam::NonlinearFactorGraph graph;
    auto reg_factor =
      gtsam::make_shared<gtsam_points::IntegratedGICPFactor_<gtsam_points::iVox, gtsam_points::PointCloud>>(gtsam::Pose3::Identity(), 0, target_ivox, frame, target_ivox);
    reg_factor->set_num_threads(num_threads);
    graph.add(reg_factor);

    gtsam_points::LevenbergMarquardtExtParams lm_params;
    // lm_params.set_verbose();
    lm_params.setMaxIterations(10);
    values = gtsam_points::LevenbergMarquardtOptimizerExt(graph, values, lm_params).optimize();

    T_odom_lidar = values.at<gtsam::Pose3>(0);
  }

  logger->debug("odometry estimated: time={} T_odom_lidar={}", raw_frame->stamp, convert_to_string(Eigen::Isometry3d(T_odom_lidar.matrix())));

  auto transformed = gtsam_points::transform(frame, Eigen::Isometry3d(T_odom_lidar.matrix()));
  target_ivox->insert(*transformed);
  Ts_odom_lidar.emplace_back(raw_frame->stamp, Eigen::Isometry3d(T_odom_lidar.matrix()));

  logger->debug("odometry updated: |Ts_odom_lidar|={} |target_ivox|={}", Ts_odom_lidar.size(), target_ivox->num_voxels());
}

}  // namespace glim
