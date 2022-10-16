#include <glim/frontend/loose_initial_state_estimation.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam_ext/ann/ivox.hpp>
#include <gtsam_ext/types/frame_cpu.hpp>
#include <gtsam_ext/factors/linear_damping_factor.hpp>
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

  gtsam::Pose3 estimated_T_odom_lidar = gtsam::Pose3::Identity();

  if (!T_odom_lidar.empty()) {
    gtsam::Pose3 init_T_odom_lidar(T_odom_lidar.back().second.matrix());

    if (T_odom_lidar.size() >= 2) {
      // Linear twist motion assumption
      Eigen::Isometry3d delta = T_odom_lidar[T_odom_lidar.size() - 2].second.inverse() * T_odom_lidar[T_odom_lidar.size() - 1].second;
      delta.linear() = Eigen::Quaterniond(delta.linear()).normalized().toRotationMatrix();
      init_T_odom_lidar = init_T_odom_lidar * gtsam::Pose3(delta.matrix());
    }

    gtsam::Values values;
    values.insert(0, init_T_odom_lidar);

    gtsam::NonlinearFactorGraph graph;
    auto factor = gtsam::make_shared<gtsam_ext::IntegratedGICPFactor_<gtsam_ext::iVox, gtsam_ext::Frame>>(gtsam::Pose3::Identity(), 0, target_ivox, frame, target_ivox);
    factor->set_num_threads(2);
    graph.add(factor);

    gtsam_ext::LevenbergMarquardtExtParams lm_params;
    // lm_params.set_verbose();
    lm_params.setMaxIterations(10);
    values = gtsam_ext::LevenbergMarquardtOptimizerExt(graph, values, lm_params).optimize();

    estimated_T_odom_lidar = values.at<gtsam::Pose3>(0);
  }

  /*
  auto viewer = guik::LightViewer::instance();
  viewer->invoke([=] {
    viewer->update_drawable(guik::anon(), glk::Primitives::coordinate_system(), guik::VertexColor(estimated_T_odom_lidar.matrix()));
    viewer->update_drawable(guik::anon(), std::make_shared<glk::PointCloudBuffer>(frame->points, frame->size()), guik::Rainbow(estimated_T_odom_lidar.matrix()));
  });
  */

  auto transformed = gtsam_ext::transform(frame, Eigen::Isometry3d(estimated_T_odom_lidar.matrix()));
  target_ivox->insert(*transformed);

  T_odom_lidar.emplace_back(raw_frame->stamp, Eigen::Isometry3d(estimated_T_odom_lidar.matrix()));
}

void LooseInitialStateEstimation::insert_imu(double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  imu_integration->insert_imu(stamp, linear_acc, angular_vel);
}

EstimationFrame::ConstPtr LooseInitialStateEstimation::initial_pose() {
  if (T_odom_lidar.empty() || T_odom_lidar.back().first - T_odom_lidar.front().first < 1.0) {
    return nullptr;
  }

  using gtsam::symbol_shorthand::B;
  using gtsam::symbol_shorthand::V;
  using gtsam::symbol_shorthand::X;

  gtsam::NonlinearFactorGraph graph;
  for (int i = 1; i < T_odom_lidar.size(); i++) {
    const auto& [t0, T_odom_lidar0] = T_odom_lidar[i - 1];
    const auto& [t1, T_odom_lidar1] = T_odom_lidar[i];

    const Eigen::Isometry3d T_odom_imu0 = T_odom_lidar0 * T_lidar_imu;
    const Eigen::Isometry3d T_odom_imu1 = T_odom_lidar1 * T_lidar_imu;
    const Eigen::Isometry3d T_imu0_imu1 = T_odom_imu0.inverse() * T_odom_imu1;

    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(i - 1), X(i), gtsam::Pose3(T_imu0_imu1.matrix()), gtsam::noiseModel::Isotropic::Precision(6, 1e3));

    int num_integrated;
    gtsam::imuBias::ConstantBias imu_bias;
    imu_integration->integrate_imu(t0, t1, imu_bias, &num_integrated);

    graph.emplace_shared<gtsam::ImuFactor>(X(i - 1), V(i - 1), X(i), V(i), B(i - 1), imu_integration->integrated_measurements());
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(B(i - 1), B(i), gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Isotropic::Precision(6, 1e6));
  }

  graph.emplace_shared<gtsam_ext::LinearDampingFactor>(X(0), 6, 1e3);
  graph.emplace_shared<gtsam::PoseTranslationPrior<gtsam::Pose3>>(X(0), gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Precision(3, 1e3));

  gtsam::Values values;
  for (int i = 0; i < T_odom_lidar.size(); i++) {
    const Eigen::Isometry3d T_odom_imu = T_odom_lidar[i].second * T_lidar_imu;
    values.insert(X(i), gtsam::Pose3(T_odom_imu.matrix()));
    values.insert(V(i), gtsam::Vector3(0.0, 0.0, 0.0));
    values.insert(B(i), gtsam::imuBias::ConstantBias());

    graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(i), gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Precision(3, 1.0));
    graph.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(i), gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Isotropic::Precision(6, 1.0));
  }

  gtsam_ext::LevenbergMarquardtExtParams lm_params;
  lm_params.callback = [&](const gtsam_ext::LevenbergMarquardtOptimizationStatus& status, const gtsam::Values& values) {
    std::cout << status.to_string() << std::endl;
    std::cout << values.at<gtsam::Pose3>(X(0)).matrix() << std::endl;
  };

  values = gtsam_ext::LevenbergMarquardtOptimizerExt(graph, values, lm_params).optimize();

  gtsam::Pose3 T_odom_imu0 = values.at<gtsam::Pose3>(X(0));
  gtsam::Pose3 T_odom_lidar0 = T_odom_imu0 * gtsam::Pose3(T_lidar_imu.inverse().matrix());

  const auto points = target_ivox->voxel_points();
  auto viewer = guik::LightViewer::instance();
  viewer->invoke([=] { viewer->update_drawable("map", std::make_shared<glk::PointCloudBuffer>(points), guik::Rainbow(T_odom_lidar0.matrix())); });

  return nullptr;
}

}  // namespace glim
