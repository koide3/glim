#include <glim/odometry/odometry_estimation_imu.hpp>

#include <spdlog/spdlog.h>

#include <Eigen/Geometry>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
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
using gtsam::symbol_shorthand::V;  // IMU velocity   (v_world_base)
using gtsam::symbol_shorthand::X;  // IMU pose       (T_world_base)

OdometryEstimationIMUParams::OdometryEstimationIMUParams() {
  // sensor config
  Config sensor_config(GlobalConfig::get_config_path("config_sensors"));

  T_base_imu = sensor_config.param<Eigen::Isometry3d>("sensors", "T_base_imu", Eigen::Isometry3d::Identity());
  T_base_gnss = sensor_config.param<Eigen::Isometry3d>("sensors", "T_base_gnss", Eigen::Isometry3d::Identity());
  T_base_lidar = sensor_config.param<Eigen::Isometry3d>("sensors", "T_base_lidar", Eigen::Isometry3d::Identity());

  imu_bias_noise = sensor_config.param<double>("sensors", "imu_bias_noise", 1e-3);
  auto bias = sensor_config.param<std::vector<double>>("sensors", "imu_bias");
  if (bias && bias->size() == 6) {
    imu_bias = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(bias->data());
  } else {
    imu_bias.setZero();
  }

  // odometry config
  Config config(GlobalConfig::get_config_path("config_odometry"));

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

  // GNSS fusion params
  gps_noise_std = config.param<double>("odometry_estimation", "gps_noise_std", 0.05);
  gps_velocity_noise_std = config.param<double>("odometry_estimation", "gps_velocity_noise_std", 0.1);
  rtk_loss_position_scale = config.param<double>("odometry_estimation", "rtk_loss_position_scale", 100.0);
  gps_z_sigma_min = config.param<double>("odometry_estimation", "gps_z_sigma_min", 0.3);

  save_imu_rate_trajectory = config.param<bool>("odometry_estimation", "save_imu_rate_trajectory", false);

  num_threads = config.param<int>("odometry_estimation", "num_threads", 4);
  num_smoother_update_threads = 1;
}

OdometryEstimationIMUParams::~OdometryEstimationIMUParams() {}

OdometryEstimationIMU::OdometryEstimationIMU(std::unique_ptr<OdometryEstimationIMUParams>&& params_) : params(std::move(params_)) {
  marginalized_cursor = 0;

  t_base_gnss = params->T_base_gnss.translation();

  if (!params->estimate_init_state || params->initialization_mode == "NAIVE") {
    auto init_estimation = new NaiveInitialStateEstimation(params->T_base_imu, params->T_base_lidar, params->imu_bias);
    if (!params->estimate_init_state) {
      Eigen::Isometry3d init_T_world_base = params->init_T_world_imu * params->T_base_imu.inverse();
      init_estimation->set_init_state(init_T_world_base, params->init_v_world_imu);
    }
    this->init_estimation.reset(init_estimation);
  } else if (params->initialization_mode == "LOOSE") {
    auto init_estimation = new LooseInitialStateEstimation(params->T_base_imu, params->T_base_lidar, params->imu_bias);
    this->init_estimation.reset(init_estimation);
  } else {
    logger->error("unknown initialization mode {}", params->initialization_mode);
  }

  IMUIntegrationParams imu_params;
  imu_params.body_P_sensor = gtsam::Pose3(params->T_base_imu.matrix());
  imu_integration.reset(new IMUIntegration(imu_params));

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

OdometryEstimationIMU::~OdometryEstimationIMU() {}

void OdometryEstimationIMU::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Callbacks::on_insert_imu(stamp, linear_acc, angular_vel);

  if (init_estimation) {
    init_estimation->insert_imu(stamp, linear_acc, angular_vel);
  }
  imu_integration->insert_imu(stamp, linear_acc, angular_vel);
}

static Eigen::Isometry3d interpolate_pose(
  const std::vector<double>& times,
  const std::vector<Eigen::Isometry3d>& poses,
  double t) {
  if (poses.size() < 2) return poses.empty() ? Eigen::Isometry3d::Identity() : poses.back();
  if (t <= times.front()) return poses.front();
  if (t >= times.back()) return poses.back();

  auto it = std::lower_bound(times.begin(), times.end(), t);
  const int i1 = std::distance(times.begin(), it);
  const int i0 = i1 - 1;
  const double tau = (t - times[i0]) / (times[i1] - times[i0]);

  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = (1.0 - tau) * poses[i0].translation() + tau * poses[i1].translation();
  result.linear() = Eigen::Quaterniond(poses[i0].linear()).slerp(tau, Eigen::Quaterniond(poses[i1].linear())).toRotationMatrix();
  return result;
}

void OdometryEstimationIMU::insert_gnss(const double stamp, const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Vector3d& var, bool is_rtk_fixed) {
  // Buffer GNSS measurement with velocity and RTK status
  gnss_buffer.push_back({stamp, pos, vel, var, is_rtk_fixed});
}

gtsam::NonlinearFactorGraph OdometryEstimationIMU::create_gnss_factor(
  const int current,
  gtsam::Values& new_values,
  const std::vector<double>& imu_pred_times,
  const std::vector<Eigen::Isometry3d>& imu_pred_poses) {
  gtsam::NonlinearFactorGraph factors;

  if (current < 1 || gnss_buffer.empty() || imu_pred_poses.empty()) {
    return factors;
  }

  const int last = current - 1;
  const double t_last = frames[last]->stamp;
  const double t_current = frames[current]->stamp;

  const Eigen::Isometry3d& T_at_current = imu_pred_poses.back();

  const gtsam::Pose3 T_world_base_current = new_values.at<gtsam::Pose3>(X(current));

  auto it = gnss_buffer.begin();
  while (it != gnss_buffer.end()) {
    const double t_gnss = it->stamp;

    if (t_gnss <= t_last) {
      it = gnss_buffer.erase(it);
      continue;
    }
    if (t_gnss > t_current) {
      break;
    }

    // IMU-based pose at GNSS timestamp (T_world_base)
    const Eigen::Isometry3d T_at_gnss = interpolate_pose(imu_pred_times, imu_pred_poses, t_gnss);

    // ---- ENU→World calibration (one-time, velocity-based) -------------------------
    // GLIM world = FLU (X=Forward, Y=Left, Z=Up).
    // GNSSPreprocessor output = ENU (X=East, Y=North, Z=Up).
    // Doppler velocity in ENU gives the vehicle heading in ENU frame.
    // Once heading is known, R_world_enu = Rz(-enu_yaw) and t_world_enu is anchored
    // to the current odometry pose so ENU and world are consistent.
    // Skip this GNSS measurement if calibration not yet done.
    const double speed_xy = it->velocity.head<2>().norm();
    if (!enu_calibrated_) {
      if (speed_xy < 1.5) {
        it = gnss_buffer.erase(it);
        continue;
      }
      // vel_enu = [East, North, Up]  →  heading CCW from East
      enu_yaw_world_x_ = std::atan2(it->velocity.y(), it->velocity.x());
      const Eigen::Matrix3d R_world_enu =
        Eigen::AngleAxisd(-enu_yaw_world_x_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      // Correct t_world_enu_ so the first GPSFactor has exactly zero residual:
      //   p_base_world = R_world_enu * antenna_enu + t_world_enu_ - R_world_base * t_base_gnss
      //   Set p_base_world = T_at_gnss.translation():
      //   → t_world_enu_ = T_at_gnss.translation() + R_world_base * t_base_gnss - R_world_enu * antenna_enu
      t_world_enu_ = T_at_gnss.translation() + T_at_gnss.linear() * t_base_gnss - R_world_enu * it->position;
      enu_calibrated_ = true;
      logger->info(
        "ENU→World calibrated from velocity: heading_enu={:.1f}° t_world_enu=[{:.3f},{:.3f},{:.3f}]",
        enu_yaw_world_x_ * 180.0 / M_PI, t_world_enu_.x(), t_world_enu_.y(), t_world_enu_.z());
    }

    // Transform GNSS antenna position from ENU to world frame
    const Eigen::Matrix3d R_world_enu =
      Eigen::AngleAxisd(-enu_yaw_world_x_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d p_antenna_world = R_world_enu * it->position + t_world_enu_;

    // Lever-arm correction: p_base_world = p_antenna_world - R_world_base * t_base_gnss
    const Eigen::Vector3d p_base_at_gnss = p_antenna_world - T_at_gnss.linear() * t_base_gnss;
    const Eigen::Vector3d p_base_at_current =
      p_base_at_gnss + (T_at_current.translation() - T_at_gnss.translation());

    // Noise from GNSS variance
    Eigen::Vector3d pos_sigmas;
    if (it->variance.norm() > 1e-6) {
      pos_sigmas << std::sqrt(it->variance[0]), std::sqrt(it->variance[1]), std::sqrt(it->variance[2]) * 2.0;
    } else {
      pos_sigmas << params->gps_noise_std, params->gps_noise_std, params->gps_noise_std * 2.0;
    }
    if (!it->is_rtk_fixed) {
      pos_sigmas *= params->rtk_loss_position_scale;
      logger->debug("GNSS RTK lost: inflating position sigma by {}x", params->rtk_loss_position_scale);
    }
    pos_sigmas[2] = std::max(pos_sigmas[2], params->gps_z_sigma_min);
    pos_sigmas = pos_sigmas.cwiseMax(0.01).cwiseMin(1000.0);
    auto gps_noise = gtsam::noiseModel::Diagonal::Sigmas(pos_sigmas);

    factors.emplace_shared<gtsam::GPSFactor>(X(current), p_base_at_current, gps_noise);

    logger->debug("GPSFactor on X({}): p_base_world=[{:.3f},{:.3f},{:.3f}] rtk={}",
                  current, p_base_at_current.x(), p_base_at_current.y(), p_base_at_current.z(),
                  it->is_rtk_fixed);

    Callbacks::on_gnss_factor_created(gnss_id++, current, p_base_at_current);

    // Store world-frame base position in frame for SubMapping/GlobalMapping propagation
    // p_base_at_current is already in world frame (ENU→world transform + lever arm applied)
    auto gnss_data = std::make_shared<GNSSData>(it->stamp, p_base_at_current, it->velocity, it->variance, it->is_rtk_fixed);
    frames[current]->custom_data["gnss"] = gnss_data;

    it = gnss_buffer.erase(it);
  }

  return factors;
}

EstimationFrame::ConstPtr OdometryEstimationIMU::insert_frame(const PreprocessedFrame::Ptr& raw_frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
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

#ifdef GTSAM_USE_TBB
    auto arena = static_cast<tbb::task_arena*>(tbb_task_arena.get());
    arena->execute([&] {
#endif
      init_estimation->insert_frame(raw_frame);
      init_state = init_estimation->initial_pose();
#ifdef GTSAM_USE_TBB
    });
#endif

    if (init_state == nullptr) {
      logger->debug("waiting for initial IMU state estimation to be finished");
      return nullptr;
    }
    init_estimation.reset();

    logger->info("initial IMU state estimation result");
    logger->info("T_world_base={}", convert_to_string(init_state->T_world_base));
    logger->info("v_world_imu={}", convert_to_string(init_state->v_world_imu));
    logger->info("imu_bias={}", convert_to_string(init_state->imu_bias));

    // Initialize the first frame
    EstimationFrame::Ptr new_frame(new EstimationFrame);
    new_frame->id = current;
    new_frame->stamp = raw_frame->stamp;

    new_frame->T_base_imu = params->T_base_imu;
    new_frame->T_base_lidar = params->T_base_lidar;
    new_frame->T_base_gnss = params->T_base_gnss;

    new_frame->set_T_world_sensor(FrameID::BASE, init_state->T_world_base);

    new_frame->v_world_imu = init_state->v_world_imu;
    new_frame->imu_bias = init_state->imu_bias;
    new_frame->raw_frame = raw_frame;

    // Transform points into BASE frame
    std::vector<Eigen::Vector4d> points_base(raw_frame->size());
    for (int i = 0; i < raw_frame->size(); i++) {
      points_base[i] = T_base_lidar * raw_frame->points[i];
    }

    std::vector<Eigen::Vector4d> normals;
    std::vector<Eigen::Matrix4d> covs;
    covariance_estimation->estimate(points_base, raw_frame->neighbors, normals, covs);

    auto frame = std::make_shared<gtsam_points::PointCloudCPU>(points_base);
    if (raw_frame->intensities.size()) {
      frame->add_intensities(raw_frame->intensities);
    }
    frame->add_covs(covs);
    frame->add_normals(normals);
    new_frame->frame = frame;
    new_frame->frame_id = FrameID::BASE;
    create_frame(new_frame);

    Callbacks::on_new_frame(new_frame);
    frames.push_back(new_frame);

    // Initialize the estimator
    gtsam::Values new_values;
    gtsam::NonlinearFactorGraph new_factors;
    gtsam::FixedLagSmootherKeyTimestampMap new_stamps;

    new_stamps[X(0)] = raw_frame->stamp;
    new_stamps[V(0)] = raw_frame->stamp;
    new_stamps[B(0)] = raw_frame->stamp;

    new_values.insert(X(0), gtsam::Pose3(new_frame->T_world_base.matrix()));
    new_values.insert(V(0), new_frame->v_world_imu);
    new_values.insert(B(0), gtsam::imuBias::ConstantBias(new_frame->imu_bias));

    // Prior for initial IMU states
    new_factors.emplace_shared<gtsam_points::LinearDampingFactor>(X(0), 6, params->init_pose_damping_scale);
    new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(0), init_state->v_world_imu, gtsam::noiseModel::Isotropic::Precision(3, 1.0));
    new_factors.emplace_shared<gtsam_points::LinearDampingFactor>(B(0), 6, 1e6);
    new_factors.add(create_factors(current, nullptr, new_values));

    update_smoother(new_factors, new_values, new_stamps);
    update_frames(current, new_factors);

    return frames.back();
  }

  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::FixedLagSmootherKeyTimestampMap new_stamps;

  const double last_stamp = frames[last]->stamp;

  const auto last_T_world_base_ = smoother->calculateEstimate<gtsam::Pose3>(X(last));
  const auto last_T_world_base = gtsam::Pose3(last_T_world_base_.rotation().normalized(), last_T_world_base_.translation());
  const auto last_v_world_imu = smoother->calculateEstimate<gtsam::Vector3>(V(last));
  const auto last_imu_bias = smoother->calculateEstimate<gtsam::imuBias::ConstantBias>(B(last));

  const gtsam::NavState last_nav_world_base(last_T_world_base, last_v_world_imu);

  // IMU integration between LiDAR scans (inter-scan)
  // NavState overload fills imu_measurements (for ImuFactor) AND records the full
  // IMU trajectory in [t_last, t_current] — used by create_gnss_factor() below.
  std::vector<double> gnss_pred_times;
  std::vector<Eigen::Isometry3d> gnss_pred_poses;
  const int imu_read_cursor = imu_integration->integrate_imu(last_stamp, raw_frame->stamp, last_nav_world_base, last_imu_bias, gnss_pred_times, gnss_pred_poses);
  // pred_times layout: [t_last, imu_0, imu_1, ..., t_current] → N IMU steps = size - 2
  const int num_imu_integrated = std::max(0, (int)gnss_pred_times.size() - 2);
  imu_integration->erase_imu_data(imu_read_cursor);
  logger->trace("num_imu_integrated={}", num_imu_integrated);

  // IMU state prediction (imu_measurements already filled by integrate_imu above)
  const gtsam::NavState predicted_nav_world_base = imu_integration->integrated_measurements().predict(last_nav_world_base, last_imu_bias);
  gtsam::Pose3 predicted_T_world_base = predicted_nav_world_base.pose();
  gtsam::Vector3 predicted_v_world_base = predicted_nav_world_base.velocity();

  // Overwrite the predicted state with the last states if no IMU data is available
  if (num_imu_integrated < 2 && last > 1) {
    const Eigen::Isometry3d T_delta = frames[last - 1]->T_world_base.inverse() * frames[last]->T_world_base;
    predicted_T_world_base = gtsam::Pose3((frames[last]->T_world_base * T_delta).matrix());
    predicted_v_world_base = frames[last]->v_world_imu;  // v_base
  }

  new_stamps[X(current)] = raw_frame->stamp;
  new_stamps[V(current)] = raw_frame->stamp;
  new_stamps[B(current)] = raw_frame->stamp;

  new_values.insert(X(current), predicted_T_world_base);
  new_values.insert(V(current), predicted_v_world_base);
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
  std::vector<double> pred_times;
  std::vector<Eigen::Isometry3d> pred_poses;
  imu_integration->integrate_imu(raw_frame->stamp, raw_frame->scan_end_time, predicted_nav_world_base, last_imu_bias, pred_times, pred_poses);

  // Create EstimationFrame
  EstimationFrame::Ptr new_frame(new EstimationFrame);
  new_frame->id = current;
  new_frame->stamp = raw_frame->stamp;

  new_frame->T_base_imu = params->T_base_imu;
  new_frame->T_base_gnss = params->T_base_gnss;
  new_frame->T_base_lidar = params->T_base_lidar;

  new_frame->set_T_world_sensor(FrameID::BASE, Eigen::Isometry3d(predicted_T_world_base.matrix()));

  new_frame->v_world_imu = predicted_v_world_base;
  new_frame->imu_bias = last_imu_bias.vector();
  new_frame->raw_frame = raw_frame;

  if (params->save_imu_rate_trajectory) {
    new_frame->imu_rate_trajectory.resize(8, pred_times.size());

    for (int i = 0; i < pred_times.size(); i++) {
      const Eigen::Vector3d trans = pred_poses[i].translation();
      const Eigen::Quaterniond quat(pred_poses[i].linear());
      new_frame->imu_rate_trajectory.col(i) << pred_times[i], trans, quat.x(), quat.y(), quat.z(), quat.w();
    }
  }

  // Deskew and tranform points into IMU frame
  auto deskewed = deskewing->deskew(params->T_base_lidar, pred_times, pred_poses, raw_frame->stamp, raw_frame->times, raw_frame->points);
  for (auto& pt : deskewed) {
    pt = params->T_base_lidar * pt;
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
  new_frame->frame_id = FrameID::BASE;
  create_frame(new_frame);

  Callbacks::on_new_frame(new_frame);
  frames.push_back(new_frame);

  new_factors.add(create_factors(current, imu_factor, new_values));

  // GNSS factors
  auto gnss_factors = create_gnss_factor(current, new_values, gnss_pred_times, gnss_pred_poses);
  if (gnss_factors.size()) {
    new_factors.add(gnss_factors);
  }

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
}

std::vector<EstimationFrame::ConstPtr> OdometryEstimationIMU::get_remaining_frames() {
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

void OdometryEstimationIMU::update_frames(int current, const gtsam::NonlinearFactorGraph& new_factors) {
  logger->trace("update frames current={} marginalized_cursor={}", current, marginalized_cursor);

  for (int i = marginalized_cursor; i < frames.size(); i++) {
    try {
      // X(i) = T_world_BASE (body_P_sensor = T_base_imu → GTSAM body frame = base_link)
      // Use set_T_world_sensor(BASE) to consistently update all transforms:
      //   T_world_base, T_world_imu = base * T_base_imu, T_world_lidar = base * T_base_lidar
      // Avoids using uninitialized T_imu_lidar that caused garbage T_world_lidar values.
      Eigen::Isometry3d T_world_base = Eigen::Isometry3d(smoother->calculateEstimate<gtsam::Pose3>(X(i)).matrix());
      Eigen::Vector3d v_world_imu = smoother->calculateEstimate<gtsam::Vector3>(V(i));
      Eigen::Matrix<double, 6, 1> imu_bias = smoother->calculateEstimate<gtsam::imuBias::ConstantBias>(B(i)).vector();

      frames[i]->set_T_world_sensor(FrameID::BASE, T_world_base);
      frames[i]->v_world_imu = v_world_imu;
      frames[i]->imu_bias = imu_bias;
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

void OdometryEstimationIMU::update_smoother(
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

void OdometryEstimationIMU::update_smoother(int count) {
  if (count <= 0) {
    return;
  }

  update_smoother(gtsam::NonlinearFactorGraph(), gtsam::Values(), std::map<std::uint64_t, double>(), count - 1);
}

}  // namespace glim
