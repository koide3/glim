#include <glim/frontend/odometry_estimation_ct.hpp>

#include <future>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam_ext/ann/ivox.hpp>
#include <gtsam_ext/ann/kdtree.hpp>
#include <gtsam_ext/types/frame_cpu.hpp>
#include <gtsam_ext/factors/integrated_ct_gicp_factor.hpp>
#include <gtsam_ext/optimizers/levenberg_marquardt_ext.hpp>

#include <glim/util/config.hpp>
#include <glim/util/console_colors.hpp>
#include <glim/frontend/callbacks.hpp>
#include <glim/common/cloud_covariance_estimation.hpp>

namespace glim {

using gtsam::symbol_shorthand::X;
using Callbacks = OdometryEstimationCallbacks;

OdometryEstimationCTParams::OdometryEstimationCTParams() {
  Config config(GlobalConfig::get_config_path("config_frontend"));

  num_threads = config.param<int>("odometry_estimation", "num_threads", 4);
  max_correspondence_distance = config.param<double>("odometry_estimation", "max_correspondence_distance", 1.0);

  ivox_resolution = config.param<double>("odometry_estimation", "ivox_resolution", 1.0);
  ivox_min_points_dist = config.param<double>("odometry_estimation", "ivox_min_points_dist", 0.05);
  ivox_lru_thresh = config.param<int>("odometry_estimation", "ivox_lru_thresh", 30);

  stiffness_scale_first = config.param<double>("odometry_estimation", "stiffness_scale_first", 1e6);
  stiffness_scale_second = config.param<double>("odometry_estimation", "stiffness_scale_second", 1e3);
  lm_max_iterations_first = config.param<int>("odometry_estimation", "lm_max_iterations_first", 5);
  lm_max_iterations_second = config.param<int>("odometry_estimation", "lm_max_iterations_second", 5);
}

OdometryEstimationCTParams::~OdometryEstimationCTParams() {}

OdometryEstimationCT::OdometryEstimationCT(const OdometryEstimationCTParams& params) : params(params) {
  v_last_current_history.push_back(gtsam::Vector6::Zero());

  covariance_estimation.reset(new CloudCovarianceEstimation);
  target_ivox.reset(new gtsam_ext::iVox(params.ivox_resolution, params.ivox_min_points_dist, params.ivox_lru_thresh));
}

OdometryEstimationCT::~OdometryEstimationCT() {}

EstimationFrame::ConstPtr OdometryEstimationCT::insert_frame(const PreprocessedFrame::Ptr& raw_frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  Callbacks::on_insert_frame(raw_frame);

  // Create a frame to hold the current estimation state
  EstimationFrame::Ptr new_frame(new EstimationFrame);
  new_frame->id = last_frame ? last_frame->id + 1 : 0;
  new_frame->stamp = raw_frame->stamp;
  new_frame->T_lidar_imu.setIdentity();
  new_frame->v_world_imu.setZero();
  new_frame->imu_bias.setZero();

  new_frame->raw_frame = raw_frame;

  new_frame->frame_id = FrameID::LIDAR;
  gtsam_ext::FrameCPU::Ptr frame_cpu(new gtsam_ext::FrameCPU(raw_frame->points));
  frame_cpu->add_times(raw_frame->times);
  frame_cpu->add_covs(covariance_estimation->estimate(raw_frame->points, raw_frame->neighbors));
  new_frame->frame = frame_cpu;

  if (!target_ivox->has_points()) {
    new_frame->set_T_world_sensor(FrameID::LIDAR, Eigen::Isometry3d::Identity());
    Callbacks::on_new_frame(new_frame);
  } else {
    // Constant linear and angular velocity assumption
    gtsam::Vector6 predicted_v_last_current = gtsam::Vector6::Zero();
    for (const auto& v : v_last_current_history) {
      predicted_v_last_current += v;
    }
    predicted_v_last_current /= v_last_current_history.size();

    // Predict the current sensor poses at the scan beginning and end
    const double dt_last_current = raw_frame->stamp - last_frame->stamp;
    const double dt_scan = raw_frame->times.back() - raw_frame->times.front();
    const gtsam::Pose3 predicted_T_world_lidar_begin = gtsam::Pose3(last_frame->T_world_lidar.matrix()) * gtsam::Pose3::Expmap(dt_last_current * predicted_v_last_current);
    const gtsam::Pose3 predicted_T_world_lidar_end = predicted_T_world_lidar_begin * gtsam::Pose3::Expmap(dt_scan * predicted_v_last_current);

    gtsam::Values values;
    values.insert(X(0), gtsam::Pose3(predicted_T_world_lidar_begin));
    values.insert(X(1), gtsam::Pose3(predicted_T_world_lidar_end));

    // Create CT-GICP factor
    gtsam::NonlinearFactorGraph graph;
    auto factor = gtsam::make_shared<gtsam_ext::IntegratedCT_GICPFactor_<gtsam_ext::iVox, gtsam_ext::Frame>>(X(0), X(1), target_ivox, new_frame->frame, target_ivox);
    factor->set_num_threads(params.num_threads);
    factor->set_max_corresponding_distance(params.max_correspondence_distance);
    graph.add(factor);

    // LM configuration
    gtsam_ext::LevenbergMarquardtExtParams lm_params;
    lm_params.setlambdaInitial(1e-10);
    lm_params.setAbsoluteErrorTol(1e-2);
    // lm_params.callback = [&](const gtsam_ext::LevenbergMarquardtOptimizationStatus& status, const gtsam::Values& values_) { std::cout << status.to_string() << std::endl; };

    try {
      // First optimization step with a large stiffness
      // graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(0), X(1), gtsam::Pose3::identity(), gtsam::noiseModel::Isotropic::Precision(6, stiffness_scale_first));
      lm_params.setMaxIterations(params.lm_max_iterations_first);
      values = gtsam_ext::LevenbergMarquardtOptimizerExt(graph, values, lm_params).optimize();

      // Second optimization step with a more elastic setting
      // graph.erase(graph.end() - 1);
      // graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(0), X(1), gtsam::Pose3::identity(), gtsam::noiseModel::Isotropic::Precision(6, stiffness_scale_second));
      lm_params.setMaxIterations(params.lm_max_iterations_second);
      values = gtsam_ext::LevenbergMarquardtOptimizerExt(graph, values, lm_params).optimize();
    } catch (std::exception& e) {
      std::cerr << console::bold_red << "error: an exception was caught during odometry estimation" << console::reset << std::endl;
      std::cerr << e.what() << std::endl;
    }

    // Calculate the linear and angular velocity and record them for the estimation of successive frames
    const gtsam::Pose3 T_last_current = values.at<gtsam::Pose3>(X(0)).inverse() * values.at<gtsam::Pose3>(X(1));
    const gtsam::Vector6 v_last_current = gtsam::Pose3::Logmap(T_last_current) / dt_last_current;
    v_last_current_history.push_back(v_last_current);
    if (v_last_current_history.size() > 3) {
      v_last_current_history.pop_front();
    }

    new_frame->set_T_world_sensor(FrameID::LIDAR, Eigen::Isometry3d(values.at<gtsam::Pose3>(X(0)).matrix()));

    // Deskew the input points and covs
    auto deskewed_points = factor->deskewed_source_points(values, true);
    auto deskewed_covs = covariance_estimation->estimate(deskewed_points, raw_frame->neighbors);
    for (int i = 0; i < deskewed_points.size(); i++) {
      new_frame->frame->points[i] = deskewed_points[i];
      new_frame->frame->covs[i] = deskewed_covs[i];
    }
    Callbacks::on_new_frame(new_frame);
  }

  auto transformed = std::make_shared<gtsam_ext::FrameCPU>(*new_frame->frame);
  for (int i = 0; i < transformed->size(); i++) {
    transformed->points[i] = new_frame->T_world_sensor() * new_frame->frame->points[i];
    transformed->covs[i] = new_frame->T_world_sensor().matrix() * new_frame->frame->covs[i] * new_frame->T_world_sensor().matrix().transpose();
  }
  target_ivox->insert(*transformed);

  last_frame = new_frame;
  marginalized_frames.push_back(new_frame);
  Callbacks::on_marginalized_frames(marginalized_frames);

  return new_frame;
}

}  // namespace glim