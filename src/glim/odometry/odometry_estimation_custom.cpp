#include <glim/odometry/odometry_estimation_custom.hpp>

#include <spdlog/spdlog.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/factors/linear_damping_factor.hpp>
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
  /*
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
      state->frame_id = FrameID::LIDAR;
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
    if (params->enable_imu) {
      for (int i = 0; i < raw_frame->size(); i++) {
        points[i] = T_imu_lidar * raw_frame->points[i];
      }
    } else {
      std::copy(raw_frame->points.begin(), raw_frame->points.end(), points.begin());
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

    return frames.back();
  }

  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::FixedLagSmootherKeyTimestampMap new_stamps;

  const double last_stamp = frames[last]->stamp;
  const auto last_T_world_imu_ = smoother->calculateEstimate<gtsam::Pose3>(X(last));
  const auto last_T_world_imu = gtsam::Pose3(last_T_world_imu_.rotation().normalized(), last_T_world_imu_.translation());
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

  new_stamps[X(current)] = raw_frame->stamp;
  new_stamps[V(current)] = raw_frame->stamp;
  new_stamps[B(current)] = raw_frame->stamp;

  new_values.insert(X(current), predicted_T_world_imu);
  new_values.insert(V(current), predicted_v_world_imu);
  new_values.insert(B(current), last_imu_bias);

  // Constant IMU bias assumption
  new_factors.add(
    gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(last), B(current), gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Isotropic::Sigma(6, params->imu_bias_noise)));
  if (params->fix_imu_bias) {
    new_factors.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(current), gtsam::imuBias::ConstantBias(params->imu_bias), gtsam::noiseModel::Isotropic::Precision(6, 1e3)));
  }

  // Create IMU factor
  gtsam::ImuFactor::shared_ptr imu_factor;
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

  if (params->save_imu_rate_trajectory) {
    new_frame->imu_rate_trajectory.resize(8, pred_imu_times.size());

    for (int i = 0; i < pred_imu_times.size(); i++) {
      const Eigen::Vector3d trans = pred_imu_poses[i].translation();
      const Eigen::Quaterniond quat(pred_imu_poses[i].linear());
      new_frame->imu_rate_trajectory.col(i) << pred_imu_times[i], trans, quat.x(), quat.y(), quat.z(), quat.w();
    }
  }

  // Deskew and tranform points into IMU frame
  auto deskewed = deskewing->deskew(T_imu_lidar, pred_imu_times, pred_imu_poses, raw_frame->stamp, raw_frame->times, raw_frame->points);
  for (auto& pt : deskewed) {
    pt = T_imu_lidar * pt;
  }

  std::vector<Eigen::Vector4d> deskewed_normals;
  std::vector<Eigen::Matrix4d> deskewed_covs;
  covariance_estimation->estimate(deskewed, raw_frame->neighbors, deskewed_normals, deskewed_covs);

  auto frame = std::make_shared<gtsam_points::PointCloudCPU>(deskewed);
  if (raw_frame->intensities.size()) {
    frame->add_intensities(raw_frame->intensities);
  }
  frame->add_covs(deskewed_covs);
  frame->add_normals(deskewed_normals);
  new_frame->frame = frame;
  new_frame->frame_id = FrameID::IMU;
  create_frame(new_frame);

  Callbacks::on_new_frame(new_frame);
  frames.push_back(new_frame);

  new_factors.add(create_factors(current, imu_factor, new_values));

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

  std::vector<EstimationFrame::ConstPtr> active_frames(frames.begin() + marginalized_cursor, frames.end());
  Callbacks::on_update_new_frame(active_frames.back());
  Callbacks::on_update_frames(active_frames);
  logger->trace("frames updated");

  if (smoother->fallbackHappened()) {
    logger->warn("odometry estimation smoother fallback happened (time={})", raw_frame->stamp);
  }

  return frames[current];
  */
}

}  // namespace glim
