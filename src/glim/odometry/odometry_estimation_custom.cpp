#include <glim/odometry/odometry_estimation_custom.hpp>

#include <spdlog/spdlog.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/factors/linear_damping_factor.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/factors/integrated_ct_gicp_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>

#include <glim/util/config.hpp>
#include <glim/util/convert_to_string.hpp>
#include <glim/common/imu_integration.hpp>
#include <glim/common/cloud_deskewing.hpp>
#include <glim/common/cloud_covariance_estimation.hpp>
#include <glim/odometry/initial_state_estimation.hpp>
#include <glim/odometry/loose_initial_state_estimation.hpp>
#include <glim/odometry/callbacks.hpp>

#ifdef GTSAM_USE_TBB
#include <tbb/task_arena.h>
#endif

#include <guik/viewer/light_viewer.hpp>

namespace glim {

using Callbacks = OdometryEstimationCallbacks;

using gtsam::symbol_shorthand::B;  // IMU bias
using gtsam::symbol_shorthand::V;  // IMU velocity   (v_world_imu)
using gtsam::symbol_shorthand::X;  // IMU pose       (T_world_imu)

OdometryEstimationCustomParams::OdometryEstimationCustomParams() {
  // sensor config
  Config sensor_config(GlobalConfig::get_config_path("config_sensors"));
  T_lidar_imu = sensor_config.param<Eigen::Isometry3d>("sensors", "T_lidar_imu", Eigen::Isometry3d::Identity());
  imu_bias_noise = sensor_config.param<double>("sensors", "imu_bias_noise", 1e-3);
  auto bias = sensor_config.param<std::vector<double>>("sensors", "imu_bias");
  if (bias && bias->size() == 6) {
    imu_bias = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(bias->data());
  } else {
    imu_bias.setZero();
  }

  // odometry config
  Config config(GlobalConfig::get_config_path("config_odometry"));

  enable_imu = config.param<bool>("odometry_estimation", "enable_imu", true);
  enable_continuous_time = config.param<bool>("odometry_estimation", "enable_continuous_time", true);

  fix_imu_bias = config.param<bool>("odometry_estimation", "fix_imu_bias", false);

  initialization_mode = config.param<std::string>("odometry_estimation", "initialization_mode", "LOOSE");
  const auto init_T_world_imu = config.param<Eigen::Isometry3d>("odometry_estimation", "init_T_world_imu");
  const auto init_v_world_imu = config.param<Eigen::Vector3d>("odometry_estimation", "init_v_world_imu");
  this->estimate_init_state = !init_T_world_imu && !init_v_world_imu;
  this->init_T_world_imu = init_T_world_imu.value_or(Eigen::Isometry3d::Identity());
  this->init_v_world_imu = init_v_world_imu.value_or(Eigen::Vector3d::Zero());
  this->init_pose_damping_scale = config.param<double>("odometry_estimation", "init_pose_damping_scale", 1e10);

  smoother_lag = config.param<double>("odometry_estimation", "smoother_lag", 5.0);
  use_isam2_dogleg = config.param<bool>("odometry_estimation", "use_isam2_dogleg", false);
  isam2_relinearize_skip = config.param<int>("odometry_estimation", "isam2_relinearize_skip", 1);
  isam2_relinearize_thresh = config.param<double>("odometry_estimation", "isam2_relinearize_thresh", 0.1);

  max_correspondence_distance = config.param<double>("odometry_estimation", "max_correspondence_distance", 1.0);

  location_consistency_inf_scale = config.param<double>("odometry_estimation", "location_consistency_inf_scale", 1e-3);
  constant_velocity_inf_scale = config.param<double>("odometry_estimation", "constant_velocity_inf_scale", 1e3);

  num_threads = config.param<int>("odometry_estimation", "num_threads", 4);
  num_smoother_update_threads = 1;
}

OdometryEstimationCustomParams::~OdometryEstimationCustomParams() {}

OdometryEstimationCustom::OdometryEstimationCustom(std::unique_ptr<OdometryEstimationCustomParams>&& params_) : params(std::move(params_)) {
  marginalized_cursor = 0;
  T_lidar_imu.setIdentity();
  T_imu_lidar.setIdentity();

  if (!params->estimate_init_state || params->initialization_mode == "NAIVE") {
    auto init_estimation = new NaiveInitialStateEstimation(params->T_lidar_imu, params->imu_bias);
    if (!params->estimate_init_state) {
      init_estimation->set_init_state(params->init_T_world_imu, params->init_v_world_imu);
    }
    this->init_estimation.reset(init_estimation);
  } else if (params->initialization_mode == "LOOSE") {
    auto init_estimation = new LooseInitialStateEstimation(params->T_lidar_imu, params->imu_bias);
    this->init_estimation.reset(init_estimation);
  } else {
    logger->error("unknown initialization mode {}", params->initialization_mode);
  }

  imu_integration.reset(new IMUIntegration);
  deskewing.reset(new CloudDeskewing);
  covariance_estimation.reset(new CloudCovarianceEstimation(params->num_threads));

  target_ivox.reset(new gtsam_points::iVox(params->max_correspondence_distance));
  target_ivox->voxel_insertion_setting().set_min_dist_in_cell(0.1);
  target_ivox->set_lru_horizon(300);
  target_ivox->set_neighbor_voxel_mode(7);

  gtsam::ISAM2Params isam2_params;
  if (params->use_isam2_dogleg) {
    isam2_params.setOptimizationParams(gtsam::ISAM2DoglegParams());
  }
  isam2_params.relinearizeSkip = params->isam2_relinearize_skip;
  isam2_params.setRelinearizeThreshold(params->isam2_relinearize_thresh);
  smoother.reset(new FixedLagSmootherExt(params->smoother_lag, isam2_params));

#ifdef GTSAM_USE_TBB
  tbb_task_arena = std::make_shared<tbb::task_arena>(params->num_smoother_update_threads);
#endif
}

OdometryEstimationCustom::~OdometryEstimationCustom() {}

void OdometryEstimationCustom::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Callbacks::on_insert_imu(stamp, linear_acc, angular_vel);

  if (init_estimation) {
    init_estimation->insert_imu(stamp, linear_acc, angular_vel);
  }
  imu_integration->insert_imu(stamp, linear_acc, angular_vel);
}

EstimationFrame::ConstPtr OdometryEstimationCustom::insert_frame(const PreprocessedFrame::Ptr& raw_frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  if (raw_frame->size()) {
    logger->trace("insert_frame points={} times={} ~ {}", raw_frame->size(), raw_frame->times.front(), raw_frame->times.back());
  } else {
    logger->warn("insert_frame points={}", raw_frame->size());
  }
  Callbacks::on_insert_frame(raw_frame);

  const int current = frames.size();
  const int last = current - 1;

  // The very first frame
  if (frames.empty()) {
    EstimationFrame::ConstPtr init_state;

    if (params->enable_imu) {
#ifdef GTSAM_USE_TBB
      auto arena = static_cast<tbb::task_arena*>(tbb_task_arena.get());
      arena->execute([&] {
#endif
        init_estimation->insert_frame(raw_frame);
        init_state = init_estimation->initial_pose();
#ifdef GTSAM_USE_TBB
      });
#endif
    } else {
      auto state = std::make_shared<EstimationFrame>();
      state->id = 0;
      state->stamp = raw_frame->stamp;
      state->T_lidar_imu = T_lidar_imu;
      state->set_T_world_sensor(FrameID::IMU, Eigen::Isometry3d::Identity());
      state->v_world_imu.setZero();
      state->imu_bias.setZero();
      state->raw_frame = raw_frame;
      state->frame_id = FrameID::IMU;
      init_state = state;
    }

    if (init_state == nullptr) {
      logger->debug("waiting for initial IMU state estimation to be finished");
      return nullptr;
    }
    init_estimation.reset();

    logger->info("initial IMU state estimation result");
    logger->info("T_world_imu={}", convert_to_string(init_state->T_world_imu));
    logger->info("v_world_imu={}", convert_to_string(init_state->v_world_imu));
    logger->info("imu_bias={}", convert_to_string(init_state->imu_bias));

    // Initialize the first frame
    EstimationFrame::Ptr new_frame(new EstimationFrame);
    new_frame->id = current;
    new_frame->stamp = raw_frame->stamp;

    T_lidar_imu = init_state->T_lidar_imu;
    T_imu_lidar = T_lidar_imu.inverse();

    new_frame->T_lidar_imu = init_state->T_lidar_imu;
    new_frame->T_world_lidar = init_state->T_world_lidar;
    new_frame->T_world_imu = init_state->T_world_imu;

    new_frame->v_world_imu = init_state->v_world_imu;
    new_frame->imu_bias = init_state->imu_bias;
    new_frame->raw_frame = raw_frame;

    // Transform points into IMU frame
    std::vector<Eigen::Vector4d> points(raw_frame->size());
    for (int i = 0; i < raw_frame->size(); i++) {
      points[i] = T_imu_lidar * raw_frame->points[i];
    }

    std::vector<Eigen::Vector4d> normals;
    std::vector<Eigen::Matrix4d> covs;
    covariance_estimation->estimate(points, raw_frame->neighbors, normals, covs);

    auto frame = std::make_shared<gtsam_points::PointCloudCPU>(points);
    if (raw_frame->intensities.size()) {
      frame->add_intensities(raw_frame->intensities);
    }
    frame->add_covs(covs);
    frame->add_normals(normals);
    new_frame->frame = frame;
    new_frame->frame_id = params->enable_imu ? FrameID::IMU : FrameID::LIDAR;

    Callbacks::on_new_frame(new_frame);
    frames.push_back(new_frame);

    // Initialize the estimator
    gtsam::Values new_values;
    gtsam::NonlinearFactorGraph new_factors;
    gtsam::FixedLagSmootherKeyTimestampMap new_stamps;

    new_stamps[X(0)] = raw_frame->stamp;
    new_values.insert(X(0), gtsam::Pose3(new_frame->T_world_imu.matrix()));
    new_factors.emplace_shared<gtsam_points::LinearDampingFactor>(X(0), 6, params->init_pose_damping_scale);

    if (params->enable_imu) {
      new_stamps[V(0)] = raw_frame->stamp;
      new_stamps[B(0)] = raw_frame->stamp;
      new_values.insert(V(0), new_frame->v_world_imu);
      new_values.insert(B(0), gtsam::imuBias::ConstantBias(new_frame->imu_bias));
      new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(0), init_state->v_world_imu, gtsam::noiseModel::Isotropic::Precision(3, 1.0));
      new_factors.emplace_shared<gtsam_points::LinearDampingFactor>(B(0), 6, 1e6);
    }

    // Prior for initial IMU states
    update_smoother(new_factors, new_values, new_stamps);
    update_frames(current, new_factors);
    update_target(current, new_frame->T_world_imu);

    return frames.back();
  }

  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::FixedLagSmootherKeyTimestampMap new_stamps;

  const double last_stamp = frames[last]->stamp;
  const auto last_T_world_imu_ = smoother->calculateEstimate<gtsam::Pose3>(X(last));
  const auto last_T_world_imu = gtsam::Pose3(last_T_world_imu_.rotation().normalized(), last_T_world_imu_.translation());
  new_stamps[X(current)] = raw_frame->stamp;
  new_values.insert(X(current), last_T_world_imu);

  gtsam::ImuFactor::shared_ptr imu_factor;
  gtsam::ImuFactor::shared_ptr intra_scan_imu_factor;
  if (params->enable_imu) {
    const auto last_v_world_imu = smoother->calculateEstimate<gtsam::Vector3>(V(last));
    const auto last_imu_bias = smoother->calculateEstimate<gtsam::imuBias::ConstantBias>(B(last));
    const gtsam::NavState last_nav_world_imu(last_T_world_imu, last_v_world_imu);

    // IMU integration between LiDAR scans (inter-scan)
    int num_imu_integrated = 0;
    const int imu_read_cursor = imu_integration->integrate_imu(last_stamp, raw_frame->stamp, last_imu_bias, &num_imu_integrated);
    imu_integration->erase_imu_data(imu_read_cursor);
    logger->trace("num_imu_integrated={}", num_imu_integrated);

    // IMU state prediction
    const gtsam::NavState predicted_nav_world_imu = imu_integration->integrated_measurements().predict(last_nav_world_imu, last_imu_bias);
    const gtsam::Pose3 predicted_T_world_imu = predicted_nav_world_imu.pose();
    const gtsam::Vector3 predicted_v_world_imu = predicted_nav_world_imu.velocity();

    new_stamps[V(current)] = raw_frame->stamp;
    new_stamps[B(current)] = raw_frame->stamp;

    new_values.insert_or_assign(X(current), predicted_T_world_imu);
    new_values.insert_or_assign(V(current), predicted_v_world_imu);
    new_values.insert_or_assign(B(current), last_imu_bias);

    // Constant IMU bias assumption
    new_factors.add(
      gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(last), B(current), gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Isotropic::Sigma(6, params->imu_bias_noise)));
    if (params->fix_imu_bias) {
      new_factors.add(
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(current), gtsam::imuBias::ConstantBias(params->imu_bias), gtsam::noiseModel::Isotropic::Precision(6, 1e3)));
    }

    // Create IMU factor
    if (num_imu_integrated >= 2) {
      imu_factor = gtsam::make_shared<gtsam::ImuFactor>(X(last), V(last), X(current), V(current), B(last), imu_integration->integrated_measurements());
      new_factors.add(imu_factor);
    } else {
      logger->warn("insufficient number of IMU data between LiDAR scans!! (odometry_estimation)");
      logger->warn("t_last={:.6f} t_current={:.6f} num_imu={}", last_stamp, raw_frame->stamp, num_imu_integrated);
      new_factors.add(gtsam::BetweenFactor<gtsam::Vector3>(V(last), V(current), gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Sigma(3, 1.0)));
    }

    // Motion prediction for deskewing (intra-scan)
    std::vector<double> pred_imu_times;
    std::vector<Eigen::Isometry3d> pred_imu_poses;
    imu_integration->integrate_imu(raw_frame->stamp, raw_frame->scan_end_time, predicted_nav_world_imu, last_imu_bias, pred_imu_times, pred_imu_poses);
    if (pred_imu_times.size() >= 4) {
      intra_scan_imu_factor = gtsam::make_shared<gtsam::ImuFactor>(X(current), V(current), X(current + 1), V(current + 1), B(current), imu_integration->integrated_measurements());
    } else {
      logger->warn("insufficient number of IMU data for intra scan constraint!! (odometry_estimation)");
    }

    // Create EstimationFrame
    EstimationFrame::Ptr new_frame(new EstimationFrame);
    new_frame->id = current;
    new_frame->stamp = raw_frame->stamp;

    new_frame->T_lidar_imu = T_lidar_imu;
    new_frame->T_world_imu = Eigen::Isometry3d(predicted_T_world_imu.matrix());
    new_frame->T_world_lidar = Eigen::Isometry3d(predicted_T_world_imu.matrix()) * T_imu_lidar;
    new_frame->v_world_imu = predicted_v_world_imu;
    new_frame->imu_bias = last_imu_bias.vector();
    new_frame->raw_frame = raw_frame;

    // Deskew and tranform points into IMU frame
    auto deskewed = deskewing->deskew(T_imu_lidar, pred_imu_times, pred_imu_poses, raw_frame->stamp, raw_frame->times, raw_frame->points);
    for (auto& pt : deskewed) {
      pt = T_imu_lidar * pt;
    }

    std::vector<Eigen::Vector4d> deskewed_normals;
    std::vector<Eigen::Matrix4d> deskewed_covs;
    covariance_estimation->estimate(deskewed, raw_frame->neighbors, deskewed_normals, deskewed_covs);

    auto frame = std::make_shared<gtsam_points::PointCloudCPU>(deskewed);
    frame->add_covs(deskewed_covs);
    frame->add_normals(deskewed_normals);
    new_frame->frame = frame;
    new_frame->frame_id = FrameID::IMU;

    Callbacks::on_new_frame(new_frame);
    frames.push_back(new_frame);
  } else {
    // Create EstimationFrame
    EstimationFrame::Ptr new_frame(new EstimationFrame);
    new_frame->id = current;
    new_frame->stamp = raw_frame->stamp;

    new_frame->T_lidar_imu = T_lidar_imu;
    new_frame->T_world_imu = Eigen::Isometry3d(last_T_world_imu.matrix());
    new_frame->T_world_lidar = Eigen::Isometry3d(last_T_world_imu.matrix()) * T_imu_lidar;
    new_frame->v_world_imu = Eigen::Vector3d::Zero();
    new_frame->imu_bias = gtsam::Vector6::Zero();
    new_frame->raw_frame = raw_frame;

    std::vector<Eigen::Vector4d> points(raw_frame->size());
    for (int i = 0; i < raw_frame->size(); i++) {
      points[i] = T_imu_lidar * raw_frame->points[i];
    }

    std::vector<Eigen::Vector4d> normals;
    std::vector<Eigen::Matrix4d> covs;
    covariance_estimation->estimate(points, raw_frame->neighbors, normals, covs);

    auto frame = std::make_shared<gtsam_points::PointCloudCPU>(points);
    frame->add_covs(covs);
    frame->add_normals(normals);
    new_frame->frame = frame;
    new_frame->frame_id = FrameID::IMU;

    Callbacks::on_new_frame(new_frame);
    frames.push_back(new_frame);
  }

  new_factors.add(create_factors(current, imu_factor, intra_scan_imu_factor, new_values));

  // Update smoother
  Callbacks::on_smoother_update(*smoother, new_factors, new_values, new_stamps);
  update_smoother(new_factors, new_values, new_stamps, 1);
  Callbacks::on_smoother_update_finish(*smoother);

  // Find out marginalized frames
  while (marginalized_cursor < current) {
    double span = frames[current]->stamp - frames[marginalized_cursor]->stamp;
    if (span < params->smoother_lag - 0.1) {
      break;
    }

    marginalized_frames.push_back(frames[marginalized_cursor]);
    frames[marginalized_cursor].reset();
    marginalized_cursor++;
  }
  Callbacks::on_marginalized_frames(marginalized_frames);

  // Update frames
  update_frames(current, new_factors);
  update_target(current, frames[current]->T_world_imu);

  std::vector<EstimationFrame::ConstPtr> active_frames(frames.begin() + marginalized_cursor, frames.end());
  Callbacks::on_update_new_frame(active_frames.back());
  Callbacks::on_update_frames(active_frames);
  logger->trace("frames updated");

  if (smoother->fallbackHappened()) {
    logger->warn("odometry estimation smoother fallback happened (time={})", raw_frame->stamp);
  }

  return frames[current];
}

gtsam::NonlinearFactorGraph OdometryEstimationCustom::create_factors(
  const int current,
  const boost::shared_ptr<gtsam::ImuFactor>& imu_factor,
  const boost::shared_ptr<gtsam::ImuFactor>& intra_scan_imu_factor,
  gtsam::Values& new_values) {
  const int last = current - 1;
  if (current == 0) {
    update_target(current, frames[current]->T_world_imu);
    return gtsam::NonlinearFactorGraph();
  }

  // Discrete-time matching
  if (!params->enable_continuous_time) {
    gtsam::Values values;
    gtsam::NonlinearFactorGraph graph;

    if (params->enable_imu) {
      values.insert_or_assign(X(current), gtsam::Pose3(frames[current]->T_world_imu.matrix()));
      values.insert_or_assign(V(current), frames[current]->v_world_imu);
      values.insert_or_assign(B(current), gtsam::imuBias::ConstantBias(frames[current]->imu_bias));

      values.insert_or_assign(X(last), gtsam::Pose3(frames[last]->T_world_imu.matrix()));
      values.insert_or_assign(V(last), frames[last]->v_world_imu);
      values.insert_or_assign(B(last), gtsam::imuBias::ConstantBias(frames[last]->imu_bias));

      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(last), gtsam::Pose3(frames[last]->T_world_imu.matrix()), gtsam::noiseModel::Isotropic::Precision(6, 1e3));
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(last), frames[last]->v_world_imu, gtsam::noiseModel::Isotropic::Precision(3, 1e3));
      graph.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
        B(last),
        gtsam::imuBias::ConstantBias(frames[last]->imu_bias),
        gtsam::noiseModel::Isotropic::Precision(6, 1e3));
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
        B(last),
        B(current),
        gtsam::imuBias::ConstantBias(),
        gtsam::noiseModel::Isotropic::Sigma(6, params->imu_bias_noise));
      graph.add(imu_factor);
    } else {
      if (current > 3) {
        const gtsam::Pose3 T0(frames[current - 2]->T_world_imu.matrix());
        const gtsam::Pose3 T1(frames[current - 1]->T_world_imu.matrix());
        const gtsam::Pose3 delta = T0.between(T1);
        const gtsam::Vector6 v = gtsam::Pose3::Logmap(delta);
        const gtsam::Pose3 pred = T1 * gtsam::Pose3::Expmap(v * 0.9);
        values.insert_or_assign(X(current), pred);
      } else {
        values.insert_or_assign(X(current), gtsam::Pose3(frames[current]->T_world_imu.matrix()));
      }
    }

    auto raw_frame = std::make_shared<gtsam_points::PointCloudCPU>(frames[current]->raw_frame->points);
    raw_frame->add_times(frames[current]->raw_frame->times);
    raw_frame->add_covs(frames[current]->frame->covs, frames[current]->frame->size());

    auto gicp_factor =
      gtsam::make_shared<gtsam_points::IntegratedGICPFactor_<gtsam_points::iVox, gtsam_points::PointCloud>>(gtsam::Pose3(), X(current), target_ivox, raw_frame, target_ivox);
    gicp_factor->set_max_correspondence_distance(params->max_correspondence_distance);
    gicp_factor->set_num_threads(params->num_threads);
    graph.add(gicp_factor);

    gtsam_points::LevenbergMarquardtExtParams lm_params;
    // lm_params.set_verbose();
    lm_params.setMaxIterations(32);
    lm_params.setAbsoluteErrorTol(0.1);

    gtsam::Pose3 last_estimate = values.at<gtsam::Pose3>(X(current));
    lm_params.termination_criteria = [&](const gtsam::Values& values) {
      const gtsam::Pose3 current_pose = values.at<gtsam::Pose3>(X(current));
      const gtsam::Pose3 delta = last_estimate.inverse() * current_pose;

      const double delta_t = delta.translation().norm();
      const double delta_r = Eigen::AngleAxisd(delta.rotation().matrix()).angle();
      last_estimate = current_pose;

      if (delta_t < 1e-10 && delta_r < 1e-10) {
        // Maybe failed to solve the linear system
        std::cerr << "failed to solve the linear system" << std::endl;
        return false;
      }

      // Convergence check
      return delta_t < 1e-3 && delta_r < 1e-3 * M_PI / 180.0;
    };

    // Optimize
    // lm_params.setDiagonalDamping(true);
    gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);

#ifdef GTSAM_USE_TBB
    auto arena = static_cast<tbb::task_arena*>(this->tbb_task_arena.get());
    arena->execute([&] {
#endif
      values = optimizer.optimize();
#ifdef GTSAM_USE_TBB
    });
#endif

    frames[current]->T_world_imu = Eigen::Isometry3d(values.at<gtsam::Pose3>(X(current)).matrix());
    new_values.insert_or_assign(X(current), gtsam::Pose3(frames[current]->T_world_imu.matrix()));

    if (params->enable_imu) {
      frames[current]->v_world_imu = values.at<gtsam::Vector3>(V(current));
      frames[current]->imu_bias = values.at<gtsam::imuBias::ConstantBias>(B(current)).vector();
      new_values.insert_or_assign(V(current), frames[current]->v_world_imu);
      new_values.insert_or_assign(B(current), gtsam::imuBias::ConstantBias(frames[current]->imu_bias));
    }
  }
  // Continuous-time matching
  else {
    gtsam::Values values;
    gtsam::NonlinearFactorGraph graph;

    if (params->enable_imu) {
      const gtsam::NavState nav(gtsam::Pose3(frames[current]->T_world_imu.matrix()), frames[current]->v_world_imu);
      const gtsam::NavState pred = intra_scan_imu_factor->preintegratedMeasurements().predict(nav, gtsam::imuBias::ConstantBias(frames[current]->imu_bias));

      values.insert_or_assign(X(last), gtsam::Pose3(frames[last]->T_world_imu.matrix()));
      values.insert_or_assign(V(last), frames[last]->v_world_imu);
      values.insert_or_assign(B(last), gtsam::imuBias::ConstantBias(frames[last]->imu_bias));
      values.insert_or_assign(X(current), gtsam::Pose3(frames[current]->T_world_imu.matrix()));
      values.insert_or_assign(V(current), frames[current]->v_world_imu);
      values.insert_or_assign(B(current), gtsam::imuBias::ConstantBias(frames[current]->imu_bias));
      values.insert_or_assign(X(current + 1), gtsam::Pose3(pred.pose().matrix()));
      values.insert_or_assign(V(current + 1), pred.velocity());

      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(last), gtsam::Pose3(frames[last]->T_world_imu.matrix()), gtsam::noiseModel::Isotropic::Precision(6, 1e6));
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(last), frames[last]->v_world_imu, gtsam::noiseModel::Isotropic::Precision(3, 1e3));
      graph.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
        B(last),
        gtsam::imuBias::ConstantBias(frames[last]->imu_bias),
        gtsam::noiseModel::Isotropic::Precision(6, 1e3));

      graph.add(imu_factor);
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
        B(last),
        B(current),
        gtsam::imuBias::ConstantBias(),
        gtsam::noiseModel::Isotropic::Sigma(6, params->imu_bias_noise));

      graph.add(intra_scan_imu_factor);
    } else {
      gtsam::Vector6 v = gtsam::Vector6::Zero();

      if (current > 3) {
        const gtsam::Pose3 T0(frames[current - 2]->T_world_imu.matrix());
        const gtsam::Pose3 T1(frames[current - 1]->T_world_imu.matrix());
        const gtsam::Pose3 delta = T0.between(T1);
        v = gtsam::Pose3::Logmap(delta) / (frames[current - 1]->stamp - frames[current - 2]->stamp);
      }

      const gtsam::Pose3 last_T_world_imu(frames[last]->T_world_imu.matrix());
      const gtsam::Pose3 beg_T_world_imu = last_T_world_imu * gtsam::Pose3::Expmap(v * (frames[current]->stamp - frames[last]->stamp));

      const double dt = frames[current]->raw_frame->times.back();
      const gtsam::Pose3 end_T_world_imu = beg_T_world_imu * gtsam::Pose3::Expmap(v * dt);

      values.insert_or_assign(X(current), beg_T_world_imu);
      values.insert_or_assign(X(current + 1), end_T_world_imu);
    }

    auto frame = std::make_shared<gtsam_points::PointCloud>();
    frame->num_points = frames[current]->raw_frame->size();
    frame->points = const_cast<Eigen::Vector4d*>(frames[current]->raw_frame->points.data());
    frame->times = const_cast<double*>(frames[current]->raw_frame->times.data());
    frame->covs = const_cast<Eigen::Matrix4d*>(frames[current]->frame->covs);

    auto cticp_factor =
      gtsam::make_shared<gtsam_points::IntegratedCT_GICPFactor_<gtsam_points::iVox, gtsam_points::PointCloud>>(X(current), X(current + 1), target_ivox, frame, target_ivox);
    cticp_factor->set_num_threads(params->num_threads);
    cticp_factor->set_max_correspondence_distance(params->max_correspondence_distance);
    graph.add(cticp_factor);

    const Eigen::Isometry3d* last_scan_end_T_world_imu = frames[last]->get_custom_data<Eigen::Isometry3d>("scan_end_T_world_imu");
    if (last_scan_end_T_world_imu) {
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        X(current),
        gtsam::Pose3(last_scan_end_T_world_imu->matrix()),
        gtsam::noiseModel::Isotropic::Precision(6, params->location_consistency_inf_scale));
    }
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      X(current),
      X(current + 1),
      gtsam::Pose3(),
      gtsam::noiseModel::Isotropic::Precision(6, params->constant_velocity_inf_scale));

    gtsam_points::LevenbergMarquardtExtParams lm_params;
    lm_params.setMaxIterations(32);
    lm_params.setAbsoluteErrorTol(0.1);
    lm_params.setlambdaInitial(1e-10);
    lm_params.setlambdaLowerBound(1e-10);
    lm_params.setlambdaUpperBound(1e-5);
    // lm_params.set_verbose();

    // Optimize
    // lm_params.setDiagonalDamping(true);
    gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);

#ifdef GTSAM_USE_TBB
    auto arena = static_cast<tbb::task_arena*>(this->tbb_task_arena.get());
    arena->execute([&] {
#endif
      values = optimizer.optimize();
#ifdef GTSAM_USE_TBB
    });
#endif

    auto deskewed = cticp_factor->deskewed_source_points(values, true);
    for (int i = 0; i < deskewed.size(); i++) {
      frames[current]->frame->points[i] = deskewed[i];
    }

    // std::atomic_bool cont = false;
    // auto viewer = guik::viewer();
    // viewer->invoke([=, &cont] { viewer->register_ui_callback("cont", [&cont] { cont = cont || ImGui::Button("continue"); }); });

    // auto target_pts = target_ivox->voxel_points();
    // Eigen::Isometry3d source_pose = Eigen::Isometry3d(values.at<gtsam::Pose3>(X(current)).matrix());

    // guik::viewer()->invoke([=, &cont] {
    //   auto viewer = guik::viewer();
    //   viewer->sub_viewer("deskewed")->update_points("target", target_pts, guik::FlatBlue());
    //   viewer->sub_viewer("deskewed")->update_points("deskewed", deskewed, guik::FlatRed(source_pose));
    // });

    // while (!cont) {
    //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }

    // viewer->invoke([=] {
    //   auto viewer = guik::viewer();
    //   viewer->remove_ui_callback("cont");
    // });

    frames[current]->T_world_imu = Eigen::Isometry3d(values.at<gtsam::Pose3>(X(current)).matrix());

    auto scan_end_T_world_imu = std::make_shared<Eigen::Isometry3d>(values.at<gtsam::Pose3>(X(current + 1)).matrix());
    frames[current]->custom_data["scan_end_T_world_imu"] = scan_end_T_world_imu;

    new_values.insert_or_assign(X(current), gtsam::Pose3(frames[current]->T_world_imu.matrix()));

    if (params->enable_imu) {
      frames[current]->v_world_imu = values.at<gtsam::Vector3>(V(current));
      frames[current]->imu_bias = values.at<gtsam::imuBias::ConstantBias>(B(current)).vector();
      new_values.insert_or_assign(V(current), frames[current]->v_world_imu);
      new_values.insert_or_assign(B(current), gtsam::imuBias::ConstantBias(frames[current]->imu_bias));
    }
  }

  const Eigen::Isometry3d T_last_current = frames[last]->T_world_imu.inverse() * frames[current]->T_world_imu;
  gtsam::NonlinearFactorGraph factors;
  factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), gtsam::Pose3(T_last_current.matrix()), gtsam::noiseModel::Isotropic::Precision(6, 1e3));
  factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(current), gtsam::Pose3(frames[current]->T_world_imu.matrix()), gtsam::noiseModel::Isotropic::Precision(6, 1e3));

  update_target(current, frames[current]->T_world_imu);

  return factors;
}

void OdometryEstimationCustom::update_frames(int current, const gtsam::NonlinearFactorGraph& new_factors) {
  logger->trace("update frames current={} marginalized_cursor={}", current, marginalized_cursor);

  for (int i = marginalized_cursor; i < frames.size(); i++) {
    try {
      Eigen::Isometry3d T_world_imu = Eigen::Isometry3d(smoother->calculateEstimate<gtsam::Pose3>(X(i)).matrix());
      frames[i]->T_world_imu = T_world_imu;
      frames[i]->T_world_lidar = T_world_imu * T_imu_lidar;

      if (params->enable_imu) {
        Eigen::Vector3d v_world_imu = smoother->calculateEstimate<gtsam::Vector3>(V(i));
        Eigen::Matrix<double, 6, 1> imu_bias = smoother->calculateEstimate<gtsam::imuBias::ConstantBias>(B(i)).vector();
        frames[i]->v_world_imu = v_world_imu;
        frames[i]->imu_bias = imu_bias;
      }
    } catch (std::out_of_range& e) {
      logger->error("caught {}", e.what());
      logger->error("current={}", current);
      logger->error("marginalized_cursor={}", marginalized_cursor);
      Callbacks::on_smoother_corruption(frames[current]->stamp);
      fallback_smoother();
      break;
    }
  }
}

std::vector<EstimationFrame::ConstPtr> OdometryEstimationCustom::get_remaining_frames() {
  // Perform a few optimization iterations at the end
  // for(int i=0; i<5; i++) {
  //   smoother->update();
  // }
  // OdometryEstimationIMU::update_frames(frames.size() - 1, gtsam::NonlinearFactorGraph());

  std::vector<EstimationFrame::ConstPtr> marginalized_frames;
  for (int i = marginalized_cursor; i < frames.size(); i++) {
    marginalized_frames.push_back(frames[i]);
  }

  Callbacks::on_marginalized_frames(marginalized_frames);

  return marginalized_frames;
}

void OdometryEstimationCustom::update_smoother(
  const gtsam::NonlinearFactorGraph& new_factors,
  const gtsam::Values& new_values,
  const std::map<std::uint64_t, double>& new_stamp,
  int update_count) {
#ifdef GTSAM_USE_TBB
  auto arena = static_cast<tbb::task_arena*>(tbb_task_arena.get());
  arena->execute([&] {
#endif
    smoother->update(new_factors, new_values, new_stamp);
    for (int i = 0; i < update_count; i++) {
      smoother->update();
    }
#ifdef GTSAM_USE_TBB
  });
#endif
}

void OdometryEstimationCustom::update_smoother(int count) {
  if (count <= 0) {
    return;
  }

  update_smoother(gtsam::NonlinearFactorGraph(), gtsam::Values(), std::map<std::uint64_t, double>(), count - 1);
}

void OdometryEstimationCustom::update_target(const int current, const Eigen::Isometry3d& T_target_imu) {
  const auto frame = frames[current]->frame;
  auto transformed = gtsam_points::transform(frame, T_target_imu);
  target_ivox->insert(*transformed);

  // Update target point cloud (just for visualization).
  // This is not necessary for mapping and can safely be removed.
  if (frames.size() % 25 == 0) {
    EstimationFrame::Ptr frame(new EstimationFrame);
    frame->id = frames.size() - 1;
    frame->stamp = frames.back()->stamp;

    frame->T_lidar_imu = frames.back()->T_lidar_imu;
    frame->T_world_lidar = frame->T_lidar_imu.inverse();
    frame->T_world_imu.setIdentity();

    frame->v_world_imu.setZero();
    frame->imu_bias.setZero();

    frame->frame_id = FrameID::IMU;

    frame->frame = std::make_shared<gtsam_points::PointCloudCPU>(target_ivox->voxel_points());

    std::vector<EstimationFrame::ConstPtr> keyframes = {frame};
    Callbacks::on_update_keyframes(keyframes);
  }
}

}  // namespace glim
