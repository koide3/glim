#include <glim/frontend/loose_initial_state_estimation.hpp>

#include <gtsam_ext/ann/ivox.hpp>
#include <gtsam_ext/types/frame_cpu.hpp>
#include <gtsam_ext/factors/integrated_gicp_factor.hpp>
#include <gtsam_ext/optimizers/levenberg_marquardt_ext.hpp>
#include <glim/common/imu_integration.hpp>
#include <glim/common/cloud_covariance_estimation.hpp>

#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace glim {

LooseInitialStateEstimation::LooseInitialStateEstimation(const Eigen::Isometry3d& T_lidar_imu, const Eigen::Matrix<double, 6, 1>& imu_bias) : T_lidar_imu(T_lidar_imu) {
  target_ivox.reset(new gtsam_ext::iVox(0.5));
  covariance_estimation.reset(new CloudCovarianceEstimation(2));
  imu_integration.reset(new glim::IMUIntegration());
}

LooseInitialStateEstimation::~LooseInitialStateEstimation() {}

void LooseInitialStateEstimation::insert_frame(const PreprocessedFrame::ConstPtr& raw_frame) {
  auto frame = std::make_shared<gtsam_ext::FrameCPU>(raw_frame->points);
  frame->add_covs(covariance_estimation->estimate(raw_frame->points, raw_frame->neighbors));

  gtsam::Pose3 lidar_pose = gtsam::Pose3::Identity();

  if (!T_odom_lidar.empty()) {
    gtsam::Pose3 init_T_odom_lidar(T_odom_lidar.back().second.matrix());

    if (T_odom_lidar.size() >= 2) {
      gtsam::Pose3 delta = gtsam::Pose3(T_odom_lidar[T_odom_lidar.size() - 2].second.matrix()).inverse() * init_T_odom_lidar;
      // init_T_odom_lidar = init_T_odom_lidar * delta;
    }

    gtsam::Values values;
    values.insert(0, init_T_odom_lidar);

    gtsam::NonlinearFactorGraph graph;
    auto factor = gtsam::make_shared<gtsam_ext::IntegratedGICPFactor_<gtsam_ext::iVox, gtsam_ext::Frame>>(gtsam::Pose3::Identity(), 0, target_ivox, frame, target_ivox);
    factor->set_num_threads(2);
    graph.add(factor);

    gtsam_ext::LevenbergMarquardtExtParams lm_params;
    lm_params.set_verbose();
    lm_params.setMaxIterations(10);
    values = gtsam_ext::LevenbergMarquardtOptimizerExt(graph, values, lm_params).optimize();

    lidar_pose = values.at<gtsam::Pose3>(0);
  }

  auto viewer = guik::LightViewer::instance();
  viewer->invoke([=] {
    viewer->update_drawable(guik::anon(), glk::Primitives::coordinate_system(), guik::VertexColor(lidar_pose.matrix()));
    viewer->update_drawable(guik::anon(), std::make_shared<glk::PointCloudBuffer>(frame->points, frame->size()), guik::Rainbow(lidar_pose.matrix()));
  });

  auto transformed = gtsam_ext::transform(frame, Eigen::Isometry3d(lidar_pose.matrix()));
  target_ivox->insert(*transformed);

  T_odom_lidar.emplace_back(raw_frame->stamp, Eigen::Isometry3d(lidar_pose.matrix()));
}

void LooseInitialStateEstimation::insert_imu(double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  imu_integration->insert_imu(stamp, linear_acc, angular_vel);
}

EstimationFrame::ConstPtr LooseInitialStateEstimation::initial_pose() {
  return nullptr;
}

}  // namespace glim
