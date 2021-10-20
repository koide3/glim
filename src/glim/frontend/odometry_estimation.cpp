#include <glim/frontend/odometry_estimation.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam_ext/cuda/cuda_device_sync.hpp>
#include <gtsam_ext/cuda/stream_temp_buffer_roundrobin.hpp>
#include <gtsam_ext/types/voxelized_frame_gpu.hpp>
#include <gtsam_ext/factors/loose_prior_factor.hpp>
#include <gtsam_ext/factors/pose3_calib_factor.hpp>
#include <gtsam_ext/factors/integrated_gicp_factor.hpp>
#include <gtsam_ext/factors/integrated_vgicp_factor.hpp>
#include <gtsam_ext/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_ext/optimizers/incremental_fixed_lag_smoother_ext.hpp>

#include <glim/util/config.hpp>
#include <glim/common/imu_integration.hpp>
#include <glim/common/cloud_deskewing.hpp>
#include <glim/common/cloud_covariance_estimation.hpp>

#include <glim/frontend/callbacks.hpp>

#include <guik/viewer/light_viewer.hpp>

namespace glim {

using Callbacks = OdometryEstimationCallbacks;

using gtsam::symbol_shorthand::B;  // IMU bias
using gtsam::symbol_shorthand::V;  // IMU velocity   (v_world_imu)
using gtsam::symbol_shorthand::X;  // IMU pose       (T_world_imu)

OdometryEstimation::OdometryEstimation() {
  Config config(GlobalConfig::get_config_path("config_frontend"));

  factor_type = config.param<std::string>("odometry_estimation", "factor_type", "VGICP_GPU");

  voxel_resolution = config.param<double>("odometry_estimation", "voxel_resolution", 0.5);
  max_num_keyframes = config.param<int>("odometry_estimation", "max_num_keyframes", 10);
  full_connection_window_size = config.param<int>("odometry_estimation", "full_connection_window_size", 3);

  keyframe_min_overlap = config.param<double>("odometry_estimation", "keyframe_min_overlap", 0.1);
  keyframe_max_overlap = config.param<double>("odometry_estimation", "keyframe_max_overlap", 0.9);

  marginalized_cursor = 0;
  enable_matching_cost_factors = true;

  T_lidar_imu.setIdentity();
  T_imu_lidar.setIdentity();
  init_estimation.reset(new NaiveInitialStateEstimation);

  imu_integration.reset(new IMUIntegration);
  deskewing.reset(new CloudDeskewing);
  covariance_estimation.reset(new CloudCovarianceEstimation);

  smoother_lag = config.param<double>("odometry_estimation", "smoother_lag", 5.0);
  gtsam::ISAM2Params isam2_params;
  if (config.param<bool>("odometry_estimation", "use_isam2_dogleg", false)) {
    isam2_params.setOptimizationParams(gtsam::ISAM2DoglegParams());
  }
  isam2_params.setRelinearizeSkip(config.param<int>("odometry_estimation", "isam2_relinearize_skip", 1));
  isam2_params.setRelinearizeThreshold(config.param<double>("odometry_estimation", "isam2_relinearize_thresh", 0.1));
  smoother.reset(new gtsam_ext::IncrementalFixedLagSmootherExt(smoother_lag, isam2_params));

  guik::LightViewer::instance()->invoke([this] {
    guik::LightViewer::instance()->register_ui_callback("call", [&] {
      bool flag = enable_matching_cost_factors;
      if(ImGui::Checkbox("enable matching cost factors", &flag)) {
        enable_matching_cost_factors = flag;
      }
    });
  });

#if BUILD_GTSAM_EXT_GPU
  stream_buffer_roundrobin.reset(new gtsam_ext::StreamTempBufferRoundRobin());
#endif
}

OdometryEstimation ::~OdometryEstimation() {}

void OdometryEstimation::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Callbacks::on_insert_imu(stamp, linear_acc, angular_vel);

  if (init_estimation) {
    init_estimation->insert_imu(stamp, linear_acc, angular_vel);
  }
  imu_integration->insert_imu(stamp, linear_acc, angular_vel);
}

EstimationFrame::ConstPtr OdometryEstimation::insert_frame(const PreprocessedFrame::Ptr& raw_frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  Callbacks::on_insert_frame(raw_frame);

  const int current = frames.size();
  const int last = current - 1;

  if (frames.empty()) {
    auto init_state = init_estimation->initial_pose();
    if (init_state == nullptr) {
      return nullptr;
    }

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

    // Transform points to IMU frame
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> points_imu(raw_frame->size());
    for (int i = 0; i < raw_frame->size(); i++) {
      points_imu[i] = T_imu_lidar * raw_frame->points[i];
    }

    auto covs = covariance_estimation->estimate(points_imu, raw_frame->neighbors);
    new_frame->frame = std::make_shared<gtsam_ext::VoxelizedFrameGPU>(voxel_resolution, points_imu, covs);
    new_frame->frame_id = "imu";

    Callbacks::on_new_frame(new_frame);
    frames.push_back(new_frame);
    imu_factors.push_back(nullptr);
    keyframes.push_back(new_frame);

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
    new_factors.add(gtsam_ext::LoosePriorFactor<gtsam::Pose3>(X(0), gtsam::Pose3(init_state->T_world_imu.matrix()), gtsam::noiseModel::Isotropic::Precision(6, 1e6)));
    new_factors.add(gtsam::PriorFactor<gtsam::Vector3>(V(0), init_state->v_world_imu, gtsam::noiseModel::Isotropic::Precision(3, 1.0)));
    new_factors.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), gtsam::imuBias::ConstantBias(init_state->imu_bias), gtsam::noiseModel::Isotropic::Precision(6, 1e2)));

    smoother->update(new_factors, new_values, new_stamps);

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
  new_factors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(last), B(current), gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Isotropic::Precision(6, 1e6)));
  // Create IMU factor
  gtsam::ImuFactor::shared_ptr imu_factor;
  if (num_imu_integrated > 2) {
    imu_factor = gtsam::make_shared<gtsam::ImuFactor>(X(last), V(last), X(current), V(current), B(last), imu_integration->integrated_measurements());
    new_factors.add(imu_factor);
  } else {
    std::cerr << "warning: insufficient number of IMU data between LiDAR scans!! (odometry_estimation)" << std::endl;
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

  // Deskew and tranform points into IMU frame
  auto deskewed = deskewing->deskew(T_imu_lidar, pred_imu_times, pred_imu_poses, raw_frame->stamp, raw_frame->times, raw_frame->points);
  for (auto& pt : deskewed) {
    pt = T_imu_lidar * pt;
  }

  auto deskewed_covs = covariance_estimation->estimate(deskewed, raw_frame->neighbors);
  new_frame->frame = std::make_shared<gtsam_ext::VoxelizedFrameGPU>(voxel_resolution, deskewed, deskewed_covs);
  new_frame->frame_id = "imu";

  Callbacks::on_new_frame(new_frame);
  frames.push_back(new_frame);
  imu_factors.push_back(imu_factor);

  // Create matching cost factors
  new_factors.add(create_matching_cost_factors(current));

  // Update smoother
  Callbacks::on_smoother_update(*smoother, new_factors, new_values);
  smoother->update(new_factors, new_values, new_stamps);

  // Find out marginalized frames
  while (marginalized_cursor < current) {
    double span = frames[current]->stamp - frames[marginalized_cursor]->stamp;
    if (span < smoother_lag - 0.1) {
      break;
    }

    marginalized_frames.push_back(frames[marginalized_cursor]);
    frames[marginalized_cursor].reset();
    imu_factors[marginalized_cursor].reset();
    marginalized_cursor++;
  }
  Callbacks::on_marginalized_frames(marginalized_frames);

  // Update frames and keyframes
  update_frames(current);
  update_keyframes(current);

  std::vector<EstimationFrame::ConstPtr> active_frames(frames.begin() + marginalized_cursor, frames.end());
  Callbacks::on_update_frames(active_frames);
  Callbacks::on_update_keyframes(keyframes);

  return frames[current];
}

std::vector<EstimationFrame::ConstPtr> OdometryEstimation::get_remaining_frames() {
  std::vector<EstimationFrame::ConstPtr> marginalized_frames;
  for (int i = marginalized_cursor; i < frames.size(); i++) {
    marginalized_frames.push_back(frames[i]);
  }

  Callbacks::on_marginalized_frames(marginalized_frames);

  return marginalized_frames;
}

void OdometryEstimation::fallback_smoother() {
  // We observed that IncrementalFixedLagSmoother occasionally marginalize values that are
  // still in the optimization window, resulting in an out_of_range exception
  // (we have to remember that it is in gtsam_unstable directory)
  // To continue estimation, we reset the smoother here with saved states
  std::cerr << "warning: smoother corrupted!!" << std::endl;
  std::cerr << "       : falling back to recover the smoother!!" << std::endl;

  std::cout << "num_frames:" << frames.size() << std::endl;
  std::cout << "mc        :" << marginalized_cursor << std::endl;

  Config config(GlobalConfig::get_config_path("config_frontend"));
  gtsam::ISAM2Params isam2_params;
  if (config.param<bool>("odometry_estimation", "use_isam2_dogleg", false)) {
    isam2_params.setOptimizationParams(gtsam::ISAM2DoglegParams());
  }
  isam2_params.setRelinearizeSkip(config.param<int>("odometry_estimation", "isam2_relinearize_skip", 1));
  isam2_params.setRelinearizeThreshold(config.param<double>("odometry_estimation", "isam2_relinearize_thresh", 0.1));
  smoother.reset(new gtsam_ext::IncrementalFixedLagSmootherExt(smoother_lag, isam2_params));

  gtsam::Values values;
  gtsam::NonlinearFactorGraph factors;
  gtsam::FixedLagSmootherKeyTimestampMap stamps;

  for (int i = marginalized_cursor; i < frames.size(); i++) {
    const auto& frame = frames[i];
    values.insert(X(i), gtsam::Pose3(frame->T_world_imu.matrix()));
    values.insert(V(i), frame->v_world_imu);
    values.insert(B(i), gtsam::imuBias::ConstantBias(frame->imu_bias));
    stamps[X(i)] = frame->stamp;
    stamps[V(i)] = frame->stamp;
    stamps[B(i)] = frame->stamp;

    factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(i), gtsam::Pose3(frame->T_world_imu.matrix()), gtsam::noiseModel::Isotropic::Precision(6, 1e3));
    factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(i), frame->v_world_imu, gtsam::noiseModel::Isotropic::Precision(3, 1e3));
    factors.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(i), gtsam::imuBias::ConstantBias(frame->imu_bias), gtsam::noiseModel::Isotropic::Precision(6, 1e3));

    if (i != marginalized_cursor) {
      factors.push_back(imu_factors[i]);
      factors.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(B(i - 1), B(i), gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Isotropic::Precision(6, 1e6));
    }

    for (int target = i - full_connection_window_size; target < i; target++) {
      if (target <= marginalized_cursor) {
        continue;
      }

      auto stream_buffer = stream_buffer_roundrobin->get_stream_buffer();
      auto& stream = stream_buffer.first;
      auto& buffer = stream_buffer.second;
      factors.emplace_shared<gtsam_ext::IntegratedVGICPFactorGPU>(X(target), X(i), frames[target]->frame, frames[i]->frame, stream, buffer);
    }
  }

  smoother->update(factors, values, stamps);
}

void OdometryEstimation::update_frames(int current) {
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
      Callbacks::on_smoother_corruption();
      fallback_smoother();
      break;
    }
  }
}

void OdometryEstimation::update_keyframes(int current) {
  if (keyframes.empty()) {
    keyframes.push_back(frames[current]);
    return;
  }

  std::vector<gtsam_ext::VoxelizedFrame::ConstPtr> keyframes_(keyframes.size());
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> delta_from_keyframes(keyframes.size());
  for (int i = 0; i < keyframes.size(); i++) {
    keyframes_[i] = keyframes[i]->frame;
    delta_from_keyframes[i] = keyframes[i]->T_world_imu.inverse() * frames[current]->T_world_imu;
  }

  const double overlap = frames[current]->frame->overlap_gpu(keyframes_, delta_from_keyframes);
  if (overlap > keyframe_max_overlap) {
    return;
  }

  const auto& new_keyframe = frames[current];
  keyframes.push_back(new_keyframe);

  if (keyframes.size() <= max_num_keyframes) {
    return;
  }

  // Remove keyframes without overlap to the new keyframe
  for (int i = 0; i < keyframes.size(); i++) {
    const Eigen::Isometry3d delta = keyframes[i]->T_world_imu.inverse() * new_keyframe->T_world_imu;
    const double overlap = new_keyframe->frame->overlap_gpu(keyframes[i]->frame, delta);
    if (overlap < keyframe_min_overlap) {
      keyframes.erase(keyframes.begin() + i);
      i--;
    }
  }

  if (keyframes.size() <= max_num_keyframes) {
    return;
  }

  // Remove the keyframe with the minimum score
  std::vector<double> scores(keyframes.size() - 1, 0.0);
  for (int i = 0; i < keyframes.size() - 1; i++) {
    const auto& keyframe = keyframes[i];
    const double overlap_latest = new_keyframe->frame->overlap_gpu(keyframe->frame, keyframe->T_world_imu.inverse() * new_keyframe->T_world_imu);

    std::vector<gtsam_ext::VoxelizedFrame::ConstPtr> other_keyframes;
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> delta_from_others;
    for (int j = 0; j < keyframes.size() - 1; j++) {
      if (i == j) {
        continue;
      }

      const auto& other = keyframes[j];
      other_keyframes.push_back(other->frame);
      delta_from_others.push_back(other->T_world_imu.inverse() * keyframe->T_world_imu);
    }

    const double overlap_others = keyframe->frame->overlap_gpu(other_keyframes, delta_from_others);
    scores[i] = overlap_latest * (1.0 - overlap_others);
  }

  double min_score = scores[0];
  int frame_to_eliminate = 0;
  for (int i = 1; i < scores.size(); i++) {
    if (scores[i] < min_score) {
      min_score = scores[i];
      frame_to_eliminate = i;
    }
  }

  keyframes.erase(keyframes.begin() + frame_to_eliminate);
}

gtsam::NonlinearFactorGraph OdometryEstimation::create_matching_cost_factors(int current) {
  const auto create_binary_factor = [this](
                                      gtsam::Key target_key,
                                      gtsam::Key source_key,
                                      const gtsam_ext::VoxelizedFrame::ConstPtr& target,
                                      const gtsam_ext::VoxelizedFrame::ConstPtr& source) -> gtsam::NonlinearFactor::shared_ptr {
    if (factor_type == "VGICP_GPU") {
      auto stream_buffer = stream_buffer_roundrobin->get_stream_buffer();
      const auto& stream = stream_buffer.first;
      const auto& buffer = stream_buffer.second;
      return gtsam::make_shared<gtsam_ext::IntegratedVGICPFactorGPU>(target_key, source_key, target, source, stream, buffer);
    } else if (factor_type == "VGICP") {
      return gtsam::make_shared<gtsam_ext::IntegratedVGICPFactor>(target_key, source_key, target, source);
    } else if (factor_type == "GICP") {
      return gtsam::make_shared<gtsam_ext::IntegratedGICPFactor>(target_key, source_key, target, source);
    }

    std::cerr << "error: unknown factor type " << factor_type << std::endl;
    return nullptr;
  };

  const auto create_unary_factor = [this](
                                     const gtsam::Pose3& fixed_target_pose,
                                     gtsam::Key source_key,
                                     const gtsam_ext::VoxelizedFrame::ConstPtr& target,
                                     const gtsam_ext::VoxelizedFrame::ConstPtr& source) -> gtsam::NonlinearFactor::shared_ptr {
    if (factor_type == "VGICP_GPU") {
      auto stream_buffer = stream_buffer_roundrobin->get_stream_buffer();
      const auto& stream = stream_buffer.first;
      const auto& buffer = stream_buffer.second;
      return gtsam::make_shared<gtsam_ext::IntegratedVGICPFactorGPU>(fixed_target_pose, source_key, target, source, stream, buffer);
    } else if (factor_type == "VGICP") {
      return gtsam::make_shared<gtsam_ext::IntegratedVGICPFactor>(fixed_target_pose, source_key, target, source);
    } else if (factor_type == "GICP") {
      return gtsam::make_shared<gtsam_ext::IntegratedGICPFactor>(fixed_target_pose, source_key, target, source);
    }

    std::cerr << "error: unknown factor type " << factor_type << std::endl;
    return nullptr;
  };

  gtsam::NonlinearFactorGraph factors;
  if (current == 0 || !enable_matching_cost_factors) {
    return factors;
  }

  // There must be factors between consecutive frames
  for (int target = current - full_connection_window_size; target < current; target++) {
    if (target < 0) {
      continue;
    }

    factors.add(create_binary_factor(X(target), X(current), frames[target]->frame, frames[current]->frame));
  }

  for (const auto& keyframe : keyframes) {
    if (keyframe->id >= current - full_connection_window_size) {
      // There already exist a factor
      continue;
    }

    auto stream_buffer = stream_buffer_roundrobin->get_stream_buffer();
    const auto& stream = stream_buffer.first;
    const auto& buffer = stream_buffer.second;

    double span = frames[current]->stamp - keyframe->stamp;
    if (span > smoother_lag - 0.1) {
      // Create unary factor
      const gtsam::Pose3 key_T_world_imu(keyframe->T_world_imu.matrix());
      factors.add(create_unary_factor(key_T_world_imu, X(current), keyframe->frame, frames[current]->frame));
    } else {
      // Create binary factor
      const int target = keyframe->id;
      auto factor = create_binary_factor(X(target), X(current), frames[target]->frame, frames[current]->frame);
      factors.add(factor);
    }
  }

  return factors;
}

}  // namespace glim