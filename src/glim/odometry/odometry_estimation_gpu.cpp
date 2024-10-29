#include <glim/odometry/odometry_estimation_gpu.hpp>

#include <spdlog/spdlog.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam_points/cuda/cuda_stream.hpp>
#include <gtsam_points/cuda/stream_temp_buffer_roundrobin.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/point_cloud_gpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/factors/linear_damping_factor.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_ext.hpp>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu.hpp>

#include <glim/util/config.hpp>
#include <glim/common/imu_integration.hpp>
#include <glim/common/cloud_deskewing.hpp>
#include <glim/common/cloud_covariance_estimation.hpp>

#include <glim/odometry/callbacks.hpp>

namespace glim {

using Callbacks = OdometryEstimationCallbacks;

using gtsam::symbol_shorthand::B;  // IMU bias
using gtsam::symbol_shorthand::V;  // IMU velocity   (v_world_imu)
using gtsam::symbol_shorthand::X;  // IMU pose       (T_world_imu)

OdometryEstimationGPUParams::OdometryEstimationGPUParams() : OdometryEstimationIMUParams() {
  // odometry config
  Config config(GlobalConfig::get_config_path("config_odometry"));

  voxel_resolution = config.param<double>("odometry_estimation", "voxel_resolution", 0.5);
  voxel_resolution_max = config.param<double>("odometry_estimation", "voxel_resolution_max", voxel_resolution);
  voxel_resolution_dmin = config.param<double>("odometry_estimation", "voxel_resolution_dmin", 4.0);
  voxel_resolution_dmax = config.param<double>("odometry_estimation", "voxel_resolution_dmax", 12.0);

  voxelmap_levels = config.param<int>("odometry_estimation", "voxelmap_levels", 2);
  voxelmap_scaling_factor = config.param<double>("odometry_estimation", "voxelmap_scaling_factor", 2.0);

  max_num_keyframes = config.param<int>("odometry_estimation", "max_num_keyframes", 10);
  full_connection_window_size = config.param<int>("odometry_estimation", "full_connection_window_size", 3);

  const std::string strategy = config.param<std::string>("odometry_estimation", "keyframe_update_strategy", "OVERLAP");
  if (strategy == "OVERLAP") {
    keyframe_strategy = KeyframeUpdateStrategy::OVERLAP;
  } else if (strategy == "DISPLACEMENT") {
    keyframe_strategy = KeyframeUpdateStrategy::DISPLACEMENT;
  } else if (strategy == "ENTROPY") {
    keyframe_strategy = KeyframeUpdateStrategy::ENTROPY;
  } else {
    spdlog::error("unknown keyframe update strategy {}", strategy);
  }

  keyframe_min_overlap = config.param<double>("odometry_estimation", "keyframe_min_overlap", 0.1);
  keyframe_max_overlap = config.param<double>("odometry_estimation", "keyframe_max_overlap", 0.9);
  keyframe_delta_trans = config.param<double>("odometry_estimation", "keyframe_delta_trans", 1.0);
  keyframe_delta_rot = config.param<double>("odometry_estimation", "keyframe_delta_rot", 0.25);
  keyframe_entropy_thresh = config.param<double>("odometry_estimation", "keyframe_entropy_thresh", 0.99);
}

OdometryEstimationGPUParams::~OdometryEstimationGPUParams() {}

OdometryEstimationGPU::OdometryEstimationGPU(const OdometryEstimationGPUParams& params) : OdometryEstimationIMU(std::make_unique<OdometryEstimationGPUParams>(params)) {
  entropy_num_frames = 0;
  entropy_running_average = 0.0;
  marginalized_cursor = 0;

  stream.reset(new gtsam_points::CUDAStream());
  stream_buffer_roundrobin.reset(new gtsam_points::StreamTempBufferRoundRobin());
}

OdometryEstimationGPU::~OdometryEstimationGPU() {
  frames.clear();
  keyframes.clear();
  smoother.reset();
}

void OdometryEstimationGPU::create_frame(EstimationFrame::Ptr& new_frame) {
  const auto params = static_cast<OdometryEstimationGPUParams*>(this->params.get());

  // Adaptively determine the voxel resolution based on the median distance
  const int max_scan_count = 256;
  const double dist_median = gtsam_points::median_distance(new_frame->frame, max_scan_count);
  const double p = std::max(0.0, std::min(1.0, (dist_median - params->voxel_resolution_dmin) / (params->voxel_resolution_dmax - params->voxel_resolution_dmin)));
  const double base_resolution = params->voxel_resolution + p * (params->voxel_resolution_max - params->voxel_resolution);

  // Create frame and voxelmaps
  new_frame->frame = gtsam_points::PointCloudGPU::clone(*new_frame->frame);
  for (int i = 0; i < params->voxelmap_levels; i++) {
    if (!new_frame->frame->size()) {
      break;
    }

    const double resolution = base_resolution * std::pow(params->voxelmap_scaling_factor, i);
    auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(resolution, 8192 * 2, 10, 1e-3, *stream);
    voxelmap->insert(*new_frame->frame);
    new_frame->voxelmaps.push_back(voxelmap);
  }
}

void OdometryEstimationGPU::update_frames(const int current, const gtsam::NonlinearFactorGraph& new_factors) {
  OdometryEstimationIMU::update_frames(current, new_factors);

  const auto params = static_cast<OdometryEstimationGPUParams*>(this->params.get());
  switch (params->keyframe_strategy) {
    case OdometryEstimationGPUParams::KeyframeUpdateStrategy::OVERLAP:
      update_keyframes_overlap(current);
      break;
    case OdometryEstimationGPUParams::KeyframeUpdateStrategy::DISPLACEMENT:
      update_keyframes_displacement(current);
      break;
    case OdometryEstimationGPUParams::KeyframeUpdateStrategy::ENTROPY:
      update_keyframes_entropy(new_factors, current);
      break;
  }

  Callbacks::on_update_keyframes(keyframes);
}

gtsam::NonlinearFactorGraph OdometryEstimationGPU::create_factors(const int current, const boost::shared_ptr<gtsam::ImuFactor>& imu_factor, gtsam::Values& new_values) {
  if (current == 0 || !frames[current]->frame->size()) {
    return gtsam::NonlinearFactorGraph();
  }

  const auto create_binary_factor = [this](
                                      gtsam::NonlinearFactorGraph& factors,
                                      gtsam::Key target_key,
                                      gtsam::Key source_key,
                                      const glim::EstimationFrame::ConstPtr& target,
                                      const glim::EstimationFrame::ConstPtr& source) {
    auto stream_buffer = stream_buffer_roundrobin->get_stream_buffer();
    const auto& stream = stream_buffer.first;
    const auto& buffer = stream_buffer.second;

    for (const auto& voxelmap : target->voxelmaps) {
      auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactorGPU>(target_key, source_key, voxelmap, source->frame, stream, buffer);
      factor->set_enable_surface_validation(true);
      factors.add(factor);
    }
  };

  const auto create_unary_factor = [this](
                                     gtsam::NonlinearFactorGraph& factors,
                                     const gtsam::Pose3& fixed_target_pose,
                                     gtsam::Key source_key,
                                     const glim::EstimationFrame::ConstPtr& target,
                                     const glim::EstimationFrame::ConstPtr& source) {
    auto stream_buffer = stream_buffer_roundrobin->get_stream_buffer();
    const auto& stream = stream_buffer.first;
    const auto& buffer = stream_buffer.second;

    for (const auto& voxelmap : target->voxelmaps) {
      auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactorGPU>(fixed_target_pose, source_key, voxelmap, source->frame, stream, buffer);
      factor->set_enable_surface_validation(true);
      factors.add(factor);
    }
  };

  const auto params = static_cast<OdometryEstimationGPUParams*>(this->params.get());

  gtsam::NonlinearFactorGraph factors;
  if (current == 0) {
    return factors;
  }

  // There must be at least one factor between consecutive frames
  for (int target = current - params->full_connection_window_size; target < current; target++) {
    if (target < 0) {
      continue;
    }

    create_binary_factor(factors, X(target), X(current), frames[target], frames[current]);
  }

  for (const auto& keyframe : keyframes) {
    if (keyframe->id >= current - params->full_connection_window_size) {
      // There already exists a factor
      continue;
    }

    auto stream_buffer = stream_buffer_roundrobin->get_stream_buffer();
    const auto& stream = stream_buffer.first;
    const auto& buffer = stream_buffer.second;

    double span = frames[current]->stamp - keyframe->stamp;
    if (span > params->smoother_lag - 0.1) {
      // Create unary factor
      const gtsam::Pose3 key_T_world_imu(keyframe->T_world_imu.matrix());
      create_unary_factor(factors, key_T_world_imu, X(current), keyframe, frames[current]);
    } else {
      // Create binary factor
      const int target = keyframe->id;
      create_binary_factor(factors, X(target), X(current), frames[target], frames[current]);
    }
  }

  return factors;
}

/**
 * @brief Keyframe management based on an overlap metric
 * @ref   Koide et al., "Globally Consistent and Tightly Coupled 3D LiDAR Inertial Mapping", ICRA2022
 */
void OdometryEstimationGPU::update_keyframes_overlap(int current) {
  const auto params = static_cast<OdometryEstimationGPUParams*>(this->params.get());

  if (!frames[current]->frame->size()) {
    return;
  }

  if (keyframes.empty()) {
    keyframes.push_back(frames[current]);
    return;
  }

  std::vector<gtsam_points::GaussianVoxelMap::ConstPtr> keyframes_(keyframes.size());
  std::vector<Eigen::Isometry3d> delta_from_keyframes(keyframes.size());
  for (int i = 0; i < keyframes.size(); i++) {
    keyframes_[i] = keyframes[i]->voxelmaps.back();
    delta_from_keyframes[i] = keyframes[i]->T_world_imu.inverse() * frames[current]->T_world_imu;
  }

  const double overlap = gtsam_points::overlap_gpu(keyframes_, frames[current]->frame, delta_from_keyframes, *stream);
  if (overlap > params->keyframe_max_overlap) {
    return;
  }

  const auto& new_keyframe = frames[current];
  keyframes.push_back(new_keyframe);

  if (keyframes.size() <= params->max_num_keyframes) {
    return;
  }

  std::vector<EstimationFrame::ConstPtr> marginalized_keyframes;

  // Remove keyframes without overlap to the new keyframe
  for (int i = 0; i < keyframes.size(); i++) {
    const Eigen::Isometry3d delta = keyframes[i]->T_world_imu.inverse() * new_keyframe->T_world_imu;
    const double overlap = gtsam_points::overlap_gpu(keyframes[i]->voxelmaps.back(), new_keyframe->frame, delta, *stream);
    if (overlap < params->keyframe_min_overlap) {
      marginalized_keyframes.push_back(keyframes[i]);
      keyframes.erase(keyframes.begin() + i);
      i--;
    }
  }

  if (keyframes.size() <= params->max_num_keyframes) {
    Callbacks::on_marginalized_keyframes(marginalized_keyframes);
    return;
  }

  // Remove the keyframe with the minimum score
  std::vector<double> scores(keyframes.size() - 1, 0.0);
  for (int i = 0; i < keyframes.size() - 1; i++) {
    const auto& keyframe = keyframes[i];
    const double overlap_latest = gtsam_points::overlap_gpu(keyframe->voxelmaps.back(), new_keyframe->frame, keyframe->T_world_imu.inverse() * new_keyframe->T_world_imu, *stream);

    std::vector<gtsam_points::GaussianVoxelMap::ConstPtr> other_keyframes;
    std::vector<Eigen::Isometry3d> delta_from_others;
    for (int j = 0; j < keyframes.size() - 1; j++) {
      if (i == j) {
        continue;
      }

      const auto& other = keyframes[j];
      other_keyframes.push_back(other->voxelmaps.back());
      delta_from_others.push_back(other->T_world_imu.inverse() * keyframe->T_world_imu);
    }

    const double overlap_others = gtsam_points::overlap_gpu(other_keyframes, keyframe->frame, delta_from_others, *stream);
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

  marginalized_keyframes.push_back(keyframes[frame_to_eliminate]);
  keyframes.erase(keyframes.begin() + frame_to_eliminate);
  Callbacks::on_marginalized_keyframes(marginalized_keyframes);
}

/**
 * @brief Keyframe management based on displacement criteria
 * @ref   Engel et al., "Direct Sparse Odometry", IEEE Trans. PAMI, 2018
 */
void OdometryEstimationGPU::update_keyframes_displacement(int current) {
  const auto params = static_cast<OdometryEstimationGPUParams*>(this->params.get());

  if (keyframes.empty()) {
    keyframes.push_back(frames[current]);
    return;
  }

  const Eigen::Isometry3d delta_from_last = keyframes.back()->T_world_imu.inverse() * frames[current]->T_world_imu;
  const double delta_trans = delta_from_last.translation().norm();
  const double delta_rot = Eigen::AngleAxisd(delta_from_last.linear()).angle();

  if (delta_trans < params->keyframe_delta_trans && delta_rot < params->keyframe_delta_rot) {
    return;
  }

  const auto& new_keyframe = frames[current];
  keyframes.push_back(new_keyframe);

  if (keyframes.size() <= params->max_num_keyframes) {
    return;
  }

  for (int i = 0; i < keyframes.size() - 1; i++) {
    const Eigen::Isometry3d delta = keyframes[i]->T_world_imu.inverse() * new_keyframe->T_world_imu;
    const double overlap = gtsam_points::overlap_gpu(keyframes[i]->voxelmaps.back(), new_keyframe->frame, delta, *stream);

    if (overlap < 0.01) {
      std::vector<EstimationFrame::ConstPtr> marginalized_keyframes;
      marginalized_keyframes.push_back(keyframes[i]);
      keyframes.erase(keyframes.begin() + i);
      Callbacks::on_marginalized_keyframes(marginalized_keyframes);
      return;
    }
  }

  const int leave_window = 2;
  const double eps = 1e-3;
  std::vector<double> scores(keyframes.size() - 1, 0.0);
  for (int i = leave_window; i < keyframes.size() - 1; i++) {
    double sum_inv_dist = 0.0;
    for (int j = 0; j < keyframes.size() - 1; j++) {
      if (i == j) {
        continue;
      }

      const double dist = (keyframes[i]->T_world_imu.translation() - keyframes[j]->T_world_imu.translation()).norm();
      sum_inv_dist += 1.0 / (dist + eps);
    }

    const double d0 = (keyframes[i]->T_world_imu.translation() - new_keyframe->T_world_imu.translation()).norm();
    scores[i] = std::sqrt(d0) * sum_inv_dist;
  }

  const auto max_score_loc = std::max_element(scores.begin(), scores.end());
  const int max_score_index = std::distance(scores.begin(), max_score_loc);

  std::vector<EstimationFrame::ConstPtr> marginalized_keyframes;
  marginalized_keyframes.push_back(keyframes[max_score_index]);
  keyframes.erase(keyframes.begin() + max_score_index);
  Callbacks::on_marginalized_keyframes(marginalized_keyframes);
}

/**
 * @brief Keyframe management based on entropy measure
 * @ref   Kuo et al., "Redesigning SLAM for Arbitrary Multi-Camera Systems", ICRA2020
 */
void OdometryEstimationGPU::update_keyframes_entropy(const gtsam::NonlinearFactorGraph& matching_cost_factors, int current) {
  const auto params = static_cast<OdometryEstimationGPUParams*>(this->params.get());

  gtsam::Values values = smoother->calculateEstimate();

  gtsam::NonlinearFactorGraph valid_factors;
  for (const auto& factor : matching_cost_factors) {
    bool valid = std::all_of(factor->keys().begin(), factor->keys().end(), [&](const gtsam::Key key) { return values.exists(key); });
    if (!valid) {
      continue;
    }

    valid_factors.push_back(factor->clone());
  }

  gtsam_points::NonlinearFactorSetGPU factor_set;
  factor_set.add(valid_factors);
  factor_set.linearize(values);
  auto linearized = valid_factors.linearize(values);

  gtsam::Matrix6 H = linearized->hessianBlockDiagonal()[X(current)];
  double negative_entropy = std::log(H.determinant());

  entropy_num_frames++;
  entropy_running_average += (negative_entropy - entropy_running_average) / entropy_num_frames;

  if (!keyframes.empty() && negative_entropy > entropy_running_average * params->keyframe_entropy_thresh) {
    return;
  }

  entropy_num_frames = 0;
  entropy_running_average = 0.0;

  const auto& new_keyframe = frames[current];
  keyframes.push_back(new_keyframe);

  if (keyframes.size() <= params->max_num_keyframes) {
    return;
  }

  std::vector<EstimationFrame::ConstPtr> marginalized_keyframes;
  marginalized_keyframes.push_back(keyframes.front());
  keyframes.erase(keyframes.begin());
  Callbacks::on_marginalized_keyframes(marginalized_keyframes);
}

}  // namespace glim
