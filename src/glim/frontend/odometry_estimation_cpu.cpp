#include <glim/frontend/odometry_estimation_cpu.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <gtsam_ext/ann/ivox.hpp>
#include <gtsam_ext/types/frame_cpu.hpp>
#include <gtsam_ext/factors/linear_damping_factor.hpp>
#include <gtsam_ext/factors/integrated_gicp_factor.hpp>
#include <gtsam_ext/factors/integrated_vgicp_factor.hpp>
#include <gtsam_ext/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_ext/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>

#include <glim/util/config.hpp>
#include <glim/util/console_colors.hpp>
#include <glim/common/imu_integration.hpp>
#include <glim/common/cloud_deskewing.hpp>
#include <glim/common/cloud_covariance_estimation.hpp>

#include <glim/frontend/callbacks.hpp>

namespace glim {

using Callbacks = OdometryEstimationCallbacks;

using gtsam::symbol_shorthand::B;  // IMU bias
using gtsam::symbol_shorthand::V;  // IMU velocity   (v_world_imu)
using gtsam::symbol_shorthand::X;  // IMU pose       (T_world_imu)

OdometryEstimationCPUParams::OdometryEstimationCPUParams() {
  // sensor config
  Config sensor_config(GlobalConfig::get_config_path("config_sensors"));
  T_lidar_imu = sensor_config.param<Eigen::Isometry3d>("sensors", "T_lidar_imu", Eigen::Isometry3d::Identity());

  auto bias = sensor_config.param<std::vector<double>>("sensors", "imu_bias");
  if (bias && bias->size() == 6) {
    imu_bias = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(bias->data());
  } else {
    imu_bias.setZero();
  }

  // frontend config
  Config config(GlobalConfig::get_config_path("config_frontend"));

  fix_imu_bias = config.param<bool>("odometry_estimation", "fix_imu_bias", false);

  const auto init_T_world_imu = config.param<Eigen::Isometry3d>("odometry_estimation", "init_T_world_imu");
  const auto init_v_world_imu = config.param<Eigen::Vector3d>("odometry_estimation", "init_v_world_imu");
  this->estimate_init_state = !init_T_world_imu && !init_v_world_imu;
  this->init_T_world_imu = init_T_world_imu.value_or(Eigen::Isometry3d::Identity());
  this->init_v_world_imu = init_v_world_imu.value_or(Eigen::Vector3d::Zero());

  registration_type = config.param<std::string>("odometry_estimation", "registration_type", "VGICP");
  max_iterations = config.param<int>("odometry_estimation", "max_iterations", 5);
  lru_thresh = config.param<int>("odometry_estimation", "lru_thresh", 100);
  target_downsampling_rate = config.param<double>("odometry_estimation", "target_downsampling_rate", 0.1);

  ivox_resolution = config.param<double>("odometry_estimation", "ivox_resolution", 0.5);
  ivox_min_dist = config.param<double>("odometry_estimation", "ivox_min_dist", 0.1);

  vgicp_resolution = config.param<double>("odometry_estimation", "vgicp_resolution", 0.2);
  vgicp_voxelmap_levels = config.param<int>("odometry_estimation", "vgicp_voxelmap_levels", 2);
  vgicp_voxelmap_scaling_factor = config.param<double>("odometry_estimation", "vgicp_voxelmap_scaling_factor", 2.0);

  smoother_lag = config.param<double>("odometry_estimation", "smoother_lag", 5.0);
  use_isam2_dogleg = config.param<bool>("odometry_estimation", "use_isam2_dogleg", false);
  isam2_relinearize_skip = config.param<int>("odometry_estimation", "isam2_relinearize_skip", 1);
  isam2_relinearize_thresh = config.param<double>("odometry_estimation", "isam2_relinearize_thresh", 0.1);

  save_imu_rate_trajectory = config.param<bool>("odometry_estimation", "save_imu_rate_trajectory", false);

  num_threads = config.param<int>("odometry_estimation", "num_threads", 4);
}

OdometryEstimationCPUParams::~OdometryEstimationCPUParams() {}

OdometryEstimationCPU::OdometryEstimationCPU(const OdometryEstimationCPUParams& params) : params(params) {
  marginalized_cursor = 0;
  T_lidar_imu.setIdentity();
  T_imu_lidar.setIdentity();

  auto init_estimation = new NaiveInitialStateEstimation(params.T_lidar_imu, params.imu_bias);
  if (!params.estimate_init_state) {
    init_estimation->set_init_state(params.init_T_world_imu, params.init_v_world_imu);
  }
  this->init_estimation.reset(init_estimation);

  if (params.registration_type == "GICP") {
    target_ivox.reset(new gtsam_ext::iVox(params.ivox_resolution, params.ivox_min_dist, params.lru_thresh));
    target_ivox->set_neighbor_voxel_mode(1);
  } else if (params.registration_type == "VGICP") {
    target_voxelmaps.resize(params.vgicp_voxelmap_levels);
    for (int i = 0; i < params.vgicp_voxelmap_levels; i++) {
      const double resolution = params.vgicp_resolution * std::pow(params.vgicp_voxelmap_scaling_factor, i);
      target_voxelmaps[i] = std::make_shared<gtsam_ext::GaussianVoxelMapCPU>(resolution);
      target_voxelmaps[i]->set_lru_thresh(params.lru_thresh);
    }
  } else {
    std::cerr << console::bold_red << "error: unknown registration type for odometry_estimation_cpu (" << params.registration_type << ")" << console::reset << std::endl;
    abort();
  }

  imu_integration.reset(new IMUIntegration);
  deskewing.reset(new CloudDeskewing);
  covariance_estimation.reset(new CloudCovarianceEstimation);

  gtsam::ISAM2Params isam2_params;
  if (params.use_isam2_dogleg) {
    isam2_params.setOptimizationParams(gtsam::ISAM2DoglegParams());
  }
  isam2_params.relinearizeSkip = params.isam2_relinearize_skip;
  isam2_params.setRelinearizeThreshold(params.isam2_relinearize_thresh);
  smoother.reset(new FixedLagSmootherExt(params.smoother_lag, isam2_params));
}

OdometryEstimationCPU::~OdometryEstimationCPU() {}

void OdometryEstimationCPU::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Callbacks::on_insert_imu(stamp, linear_acc, angular_vel);

  if (init_estimation) {
    init_estimation->insert_imu(stamp, linear_acc, angular_vel);
  }
  imu_integration->insert_imu(stamp, linear_acc, angular_vel);
}

EstimationFrame::ConstPtr OdometryEstimationCPU::insert_frame(const PreprocessedFrame::Ptr& raw_frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  Callbacks::on_insert_frame(raw_frame);

  const int current = frames.size();
  const int last = current - 1;

  if (frames.empty()) {
    auto init_state = init_estimation->initial_pose();
    if (init_state == nullptr) {
      return nullptr;
    }
    init_estimation.reset();

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
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> points_imu(raw_frame->size());
    for (int i = 0; i < raw_frame->size(); i++) {
      points_imu[i] = T_imu_lidar * raw_frame->points[i];
    }

    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> normals;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> covs;
    covariance_estimation->estimate(points_imu, raw_frame->neighbors, normals, covs);

    auto frame = std::make_shared<gtsam_ext::FrameCPU>(points_imu);
    frame->add_covs(covs);
    frame->add_normals(normals);
    new_frame->frame = frame;
    new_frame->frame_id = FrameID::IMU;

    Callbacks::on_new_frame(new_frame);
    frames.push_back(new_frame);

    // Initialize the estimator
    gtsam::Values new_values;
    gtsam::NonlinearFactorGraph new_factors;
    gtsam::FixedLagSmootherKeyTimestampMap new_stamps;

    new_stamps[X(0)] = raw_frame->stamp;
    new_stamps[V(0)] = raw_frame->stamp;
    new_stamps[B(0)] = raw_frame->stamp;

    new_values.insert(X(0), gtsam::Pose3(new_frame->T_world_imu.matrix()));
    new_values.insert(V(0), new_frame->v_world_imu);
    new_values.insert(B(0), gtsam::imuBias::ConstantBias(new_frame->imu_bias));

    // Prior for initial IMU states
    new_factors.emplace_shared<gtsam_ext::LinearDampingFactor>(X(0), 6, 1e10);
    new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(0), init_state->v_world_imu, gtsam::noiseModel::Isotropic::Precision(3, 1.0));
    new_factors.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
      B(0),
      gtsam::imuBias::ConstantBias(init_state->imu_bias),
      gtsam::noiseModel::Isotropic::Precision(6, 1e2));

    smoother->update(new_factors, new_values, new_stamps);
    update_target(frames[current]->T_world_imu, frames[current]->frame);

    return frames.back();
  }

  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::FixedLagSmootherKeyTimestampMap new_stamps;

  const double last_stamp = frames[last]->stamp;
  const auto last_T_world_imu = smoother->calculateEstimate<gtsam::Pose3>(X(last));
  const auto last_v_world_imu = smoother->calculateEstimate<gtsam::Vector3>(V(last));
  const auto last_imu_bias = smoother->calculateEstimate<gtsam::imuBias::ConstantBias>(B(last));
  const gtsam::NavState last_nav_world_imu(last_T_world_imu, last_v_world_imu);

  // IMU integration between LiDAR scans (inter-scan)
  int num_imu_integrated = 0;
  const int imu_read_cursor = imu_integration->integrate_imu(last_stamp, raw_frame->stamp, last_imu_bias, &num_imu_integrated);
  imu_integration->erase_imu_data(imu_read_cursor);

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
  new_factors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(last), B(current), gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Isotropic::Precision(6, 1e10)));
  if (params.fix_imu_bias) {
    new_factors.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(current), gtsam::imuBias::ConstantBias(params.imu_bias), gtsam::noiseModel::Isotropic::Precision(6, 1e6)));
  }

  // Create IMU factor
  gtsam::ImuFactor::shared_ptr imu_factor;
  if (num_imu_integrated >= 2) {
    imu_factor = gtsam::make_shared<gtsam::ImuFactor>(X(last), V(last), X(current), V(current), B(last), imu_integration->integrated_measurements());
    new_factors.add(imu_factor);
  } else {
    std::cerr << console::yellow << "warning: insufficient number of IMU data between LiDAR scans!! (odometry_estimation)" << console::reset << std::endl;
    std::cerr << console::yellow << boost::format("       : t_last=%.6f t_current=%.6f num_imu=%d") % last_stamp % raw_frame->stamp % num_imu_integrated << console::reset
              << std::endl;
    new_factors.add(gtsam::BetweenFactor<gtsam::Vector3>(V(last), V(current), gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Sigma(3, 1.0)));
  }

  // Motion prediction for deskewing (intra-scan)
  std::vector<double> pred_imu_times;
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> pred_imu_poses;
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

  if (params.save_imu_rate_trajectory) {
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

  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> deskewed_normals;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> deskewed_covs;
  covariance_estimation->estimate(deskewed, raw_frame->neighbors, deskewed_normals, deskewed_covs);

  auto frame = std::make_shared<gtsam_ext::FrameCPU>(deskewed);
  frame->add_covs(deskewed_covs);
  frame->add_normals(deskewed_normals);
  new_frame->frame = frame;
  new_frame->frame_id = FrameID::IMU;

  Callbacks::on_new_frame(new_frame);
  frames.push_back(new_frame);

  auto factors = create_factors(current, imu_factor);
  new_factors.add(factors);

  // Update smoother
  Callbacks::on_smoother_update(*smoother, new_factors, new_values, new_stamps);
  smoother->update(new_factors, new_values, new_stamps);

  // Find out marginalized frames
  while (marginalized_cursor < current) {
    double span = frames[current]->stamp - frames[marginalized_cursor]->stamp;
    if (span < params.smoother_lag - 0.1) {
      break;
    }

    marginalized_frames.push_back(frames[marginalized_cursor]);
    frames[marginalized_cursor].reset();
    marginalized_cursor++;
  }
  Callbacks::on_marginalized_frames(marginalized_frames);

  // Update frames
  update_frames(current);

  std::vector<EstimationFrame::ConstPtr> active_frames(frames.begin() + marginalized_cursor, frames.end());
  Callbacks::on_update_frames(active_frames);

  return frames[current];
}

std::vector<EstimationFrame::ConstPtr> OdometryEstimationCPU::get_remaining_frames() {
  std::vector<EstimationFrame::ConstPtr> marginalized_frames;
  for (int i = marginalized_cursor; i < frames.size(); i++) {
    marginalized_frames.push_back(frames[i]);
  }

  Callbacks::on_marginalized_frames(marginalized_frames);

  return marginalized_frames;
}

gtsam::NonlinearFactorGraph OdometryEstimationCPU::create_factors(const int current, const boost::shared_ptr<gtsam::ImuFactor>& imu_factor) {
  const int last = current - 1;

  gtsam::NonlinearFactorGraph matching_cost_factors;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;

  const gtsam::Pose3 last_T_world_imu(frames[last]->T_world_imu.matrix());
  const gtsam::Vector3 last_v_world_imu = frames[last]->v_world_imu;
  const gtsam::imuBias::ConstantBias last_imu_bias(frames[last]->imu_bias);

  values.insert(X(last), gtsam::Pose3(last_T_world_imu));
  values.insert(X(current), gtsam::Pose3(frames[current]->T_world_imu.matrix()));

  values.insert(V(last), last_v_world_imu);
  values.insert(V(current), frames[current]->v_world_imu);
  values.insert(B(last), last_imu_bias);
  values.insert(B(current), gtsam::imuBias::ConstantBias(frames[current]->imu_bias));

  graph.add(imu_factor);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(last), last_T_world_imu, gtsam::noiseModel::Isotropic::Precision(6, 1e6));
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(last), last_v_world_imu, gtsam::noiseModel::Isotropic::Precision(3, 1e6));
  graph.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(last), last_imu_bias, gtsam::noiseModel::Isotropic::Precision(6, 1e6));
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(B(last), B(current), gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Isotropic::Precision(6, 1e6));

  if (params.registration_type == "GICP") {
    auto gicp_factor =
      gtsam::make_shared<gtsam_ext::IntegratedGICPFactor_<gtsam_ext::iVox, gtsam_ext::Frame>>(gtsam::Pose3(), X(current), target_ivox, frames[current]->frame, target_ivox);
    gicp_factor->set_max_corresponding_distance(params.ivox_resolution * 2.0);
    gicp_factor->set_num_threads(params.num_threads);
    matching_cost_factors.add(gicp_factor);
  } else if (params.registration_type == "VGICP") {
    for (const auto& voxelmap : target_voxelmaps) {
      auto vgicp_factor = gtsam::make_shared<gtsam_ext::IntegratedVGICPFactor>(gtsam::Pose3(), X(current), voxelmap, frames[current]->frame);
      vgicp_factor->set_num_threads(params.num_threads);
      matching_cost_factors.add(vgicp_factor);
    }
  }

  graph.add(matching_cost_factors);

  gtsam_ext::LevenbergMarquardtExtParams lm_params;
  lm_params.setMaxIterations(params.max_iterations);
  // lm_params.set_verbose();
  // lm_params.setDiagonalDamping(true);
  gtsam_ext::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);
  values = optimizer.optimize();

  const Eigen::Isometry3d T_world_imu = Eigen::Isometry3d(values.at<gtsam::Pose3>(X(current)).matrix());
  const auto subsampled = gtsam_ext::random_sampling(frames[current]->frame, params.target_downsampling_rate, mt);
  update_target(T_world_imu, subsampled);

  gtsam::NonlinearFactorGraph factors;

  for (const auto& factor : matching_cost_factors) {
    factors.emplace_shared<gtsam::LinearContainerFactor>(factor->linearize(values), values);
  }

  // Eigen::Isometry3d T_last_current = frames[last]->T_world_imu.inverse() * T_world_imu;
  // T_last_current.linear() = Eigen::Quaterniond(T_last_current.linear()).normalized().toRotationMatrix();
  // factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), gtsam::Pose3(T_last_current.matrix()), gtsam::noiseModel::Isotropic::Precision(6, 1e6));

  return factors;
}

void OdometryEstimationCPU::update_target(const Eigen::Isometry3d& T_world_frame, const gtsam_ext::Frame::ConstPtr& frame) {
  auto transformed = std::make_shared<gtsam_ext::FrameCPU>();
  transformed->num_points = frame->size();
  transformed->points_storage.resize(frame->size());
  transformed->covs_storage.resize(frame->size());
  transformed->points = transformed->points_storage.data();
  transformed->covs = transformed->covs_storage.data();

  for (int i = 0; i < frame->size(); i++) {
    transformed->points[i] = T_world_frame * frame->points[i];
    transformed->covs[i] = T_world_frame.matrix() * frame->covs[i] * T_world_frame.matrix().transpose();
  }

  if (params.registration_type == "GICP") {
    target_ivox->insert(*transformed);
  } else if (params.registration_type == "VGICP") {
    for (auto& target_voxelmap : target_voxelmaps) {
      target_voxelmap->insert(*transformed);
    }
  }

  if (frames.size() % 50 == 0) {
    EstimationFrame::Ptr frame(new EstimationFrame);
    frame->id = frames.size() - 1;
    frame->stamp = frames.back()->stamp;

    frame->T_lidar_imu = frames.back()->T_lidar_imu;
    frame->T_world_lidar = frame->T_lidar_imu.inverse();
    frame->T_world_imu.setIdentity();

    frame->v_world_imu.setZero();
    frame->imu_bias.setZero();

    frame->frame_id = FrameID::IMU;

    if (params.registration_type == "GICP") {
      frame->frame = std::make_shared<gtsam_ext::FrameCPU>(target_ivox->voxel_points());
    } else if (params.registration_type == "VGICP") {
      std::vector<Eigen::Vector4d> points;
      for (const auto& voxel : target_voxelmaps[0]->voxels) {
        points.push_back(voxel.second->mean);
      }
      frame->frame = std::make_shared<gtsam_ext::FrameCPU>(points);
    }

    std::vector<EstimationFrame::ConstPtr> keyframes = {frame};
    Callbacks::on_update_keyframes(keyframes);

    if (target_ivox_frame) {
      std::vector<EstimationFrame::ConstPtr> marginalized_keyframes = {target_ivox_frame};
      Callbacks::on_marginalized_keyframes(marginalized_keyframes);
    }

    target_ivox_frame = frame;
  }
}

void OdometryEstimationCPU::update_frames(int current) {
  for (int i = marginalized_cursor; i < frames.size(); i++) {
    try {
      Eigen::Isometry3d T_world_imu = Eigen::Isometry3d(smoother->calculateEstimate<gtsam::Pose3>(X(i)).matrix());
      Eigen::Vector3d v_world_imu = smoother->calculateEstimate<gtsam::Vector3>(V(i));
      Eigen::Matrix<double, 6, 1> imu_bias = smoother->calculateEstimate<gtsam::imuBias::ConstantBias>(B(i)).vector();

      frames[i]->T_world_imu = T_world_imu;
      frames[i]->T_world_lidar = T_world_imu * T_imu_lidar;
      frames[i]->v_world_imu = v_world_imu;
      frames[i]->imu_bias = imu_bias;
    } catch (std::out_of_range& e) {
      std::cerr << "caught " << e.what() << std::endl;
      std::cerr << "current:" << current << std::endl;
      std::cerr << "marginalized_cursor:" << marginalized_cursor << std::endl;
      Callbacks::on_smoother_corruption(frames[current]->stamp);
      // fallback_smoother();
      break;
    }
  }
}

}  // namespace glim
