#include <glim/odometry/loose_initial_state_estimation.hpp>

#include <sstream>
#include <spdlog/spdlog.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/factors/linear_damping_factor.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>

#include <glim/util/config.hpp>
#include <glim/util/convert_to_string.hpp>
#include <glim/common/imu_integration.hpp>
#include <glim/common/cloud_covariance_estimation.hpp>
#include <glim/odometry/callbacks.hpp>

namespace glim {

LooseInitialStateEstimation::LooseInitialStateEstimation(const Eigen::Isometry3d& T_lidar_imu, const Eigen::Matrix<double, 6, 1>& imu_bias) : T_lidar_imu(T_lidar_imu) {
  glim::Config config(glim::GlobalConfig::get_config_path("config_odometry"));
  num_threads = config.param("odometry_estimation", "num_threads", 2);
  window_size = config.param("odometry_estimation", "initialization_window_size", 1.0);

  target_ivox.reset(new gtsam_points::iVox(1.0));
  covariance_estimation.reset(new CloudCovarianceEstimation(num_threads));
  imu_integration.reset(new glim::IMUIntegration());
}

LooseInitialStateEstimation::~LooseInitialStateEstimation() {}

void LooseInitialStateEstimation::insert_frame(const PreprocessedFrame::ConstPtr& raw_frame) {
  if (raw_frame->size() < 50) {
    logger->warn("skip initial state estimation for a frame with too few points ({} points)", raw_frame->size());
    return;
  }

  auto frame = std::make_shared<gtsam_points::PointCloudCPU>(raw_frame->points);
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
    auto factor =
      gtsam::make_shared<gtsam_points::IntegratedGICPFactor_<gtsam_points::iVox, gtsam_points::PointCloud>>(gtsam::Pose3::Identity(), 0, target_ivox, frame, target_ivox);
    factor->set_num_threads(num_threads);
    graph.add(factor);

    gtsam_points::LevenbergMarquardtExtParams lm_params;
    // lm_params.set_verbose();
    lm_params.setMaxIterations(10);
    values = gtsam_points::LevenbergMarquardtOptimizerExt(graph, values, lm_params).optimize();

    estimated_T_odom_lidar = values.at<gtsam::Pose3>(0);
  }

  auto transformed = gtsam_points::transform(frame, Eigen::Isometry3d(estimated_T_odom_lidar.matrix()));
  target_ivox->insert(*transformed);

  T_odom_lidar.emplace_back(raw_frame->stamp, Eigen::Isometry3d(estimated_T_odom_lidar.matrix()));

  Eigen::Isometry3d pose(estimated_T_odom_lidar.matrix());
  IMUStateInitializationCallbacks::on_updated(raw_frame, pose);
}

void LooseInitialStateEstimation::insert_imu(double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  imu_integration->insert_imu(stamp, linear_acc, angular_vel);
}

EstimationFrame::ConstPtr LooseInitialStateEstimation::initial_pose() {
  if (T_odom_lidar.empty() || T_odom_lidar.back().first - T_odom_lidar.front().first < window_size) {
    return nullptr;
  }

  logger->info("estimate initial IMU state");

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
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(B(i - 1), B(i), gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Isotropic::Precision(6, 1e1));
  }

  graph.emplace_shared<gtsam_points::LinearDampingFactor>(X(0), (gtsam::Vector6() << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0).finished() * 1e6);
  graph.emplace_shared<gtsam::PoseTranslationPrior<gtsam::Pose3>>(X(0), gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Precision(3, 1e3));

  const auto& imu_data = imu_integration->imu_data_in_queue();
  int imu_cursor = 0;

  Eigen::Vector3d sum_acc_odom = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> acc_odom(T_odom_lidar.size());
  for (int i = 0; i < T_odom_lidar.size(); i++) {
    while (imu_cursor < imu_data.size() - 1 && imu_data[imu_cursor][0] < T_odom_lidar[i].first) {
      imu_cursor++;
    }

    const Eigen::Vector3d acc_local = imu_data[imu_cursor].middleRows<3>(1);
    sum_acc_odom += (T_odom_lidar[i].second * T_lidar_imu).linear() * acc_local.normalized();
  }

  Eigen::Isometry3d init_T_world_odom = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d acc_dir = sum_acc_odom.normalized();
  if (acc_dir.z() < 0.99) {
    const Eigen::Vector3d acc_world = Eigen::Vector3d::UnitZ();

    const Eigen::Vector3d v = acc_dir.cross(acc_world);
    const double s = v.norm();
    const double c = acc_dir.dot(acc_world);

    Eigen::Matrix3d skew = gtsam::SO3::Hat(v);
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() + skew + skew * skew * (1 - c) / (s * s);
    init_T_world_odom.linear() = R;
  }

  gtsam::Values values;
  for (int i = 0; i < T_odom_lidar.size(); i++) {
    const Eigen::Isometry3d T_world_imu = init_T_world_odom * T_odom_lidar[i].second * T_lidar_imu;
    values.insert(X(i), gtsam::Pose3(T_world_imu.matrix()));
    values.insert(V(i), gtsam::Vector3(0.0, 0.0, 0.0));
    values.insert(B(i), gtsam::imuBias::ConstantBias());

    graph.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(i), gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Isotropic::Precision(6, 10.0));
  }

  gtsam::LevenbergMarquardtParams lm_params;
  lm_params.setVerbosityLM("SUMMARY");
  values = gtsam::LevenbergMarquardtOptimizer(graph, values, lm_params).optimize();

  gtsam::Pose3 T_odom_imu0 = values.at<gtsam::Pose3>(X(0));
  gtsam::Pose3 T_odom_lidar0 = T_odom_imu0 * gtsam::Pose3(T_lidar_imu.inverse().matrix());

  EstimationFrame::Ptr estimated(new EstimationFrame);
  estimated->id = -1;
  estimated->stamp = T_odom_lidar.back().first;
  estimated->T_lidar_imu = T_lidar_imu;
  estimated->v_world_imu = values.at<gtsam::Vector3>(V(T_odom_lidar.size() - 1));
  estimated->imu_bias = values.at<gtsam::imuBias::ConstantBias>(B(T_odom_lidar.size() - 1)).vector();

  estimated->T_world_imu = Eigen::Isometry3d(values.at<gtsam::Pose3>(X(T_odom_lidar.size() - 1)).matrix());
  estimated->T_world_lidar = estimated->T_world_imu * T_lidar_imu.inverse();

  IMUStateInitializationCallbacks::on_finished(estimated);

  return estimated;
}

}  // namespace glim
