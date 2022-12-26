#include <glim/backend/sub_mapping.hpp>

#include <spdlog/spdlog.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <gtsam_ext/types/frame.hpp>
#include <gtsam_ext/types/frame_cpu.hpp>
#include <gtsam_ext/types/frame_gpu.hpp>
#include <gtsam_ext/types/gaussian_voxelmap.hpp>
#include <gtsam_ext/types/voxelized_frame_cpu.hpp>
#include <gtsam_ext/types/voxelized_frame_gpu.hpp>
#include <gtsam_ext/factors/integrated_gicp_factor.hpp>
#include <gtsam_ext/factors/integrated_vgicp_factor.hpp>
#include <gtsam_ext/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_ext/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_ext/cuda/cuda_stream.hpp>
#include <gtsam_ext/cuda/stream_temp_buffer_roundrobin.hpp>

#include <glim/util/config.hpp>
#include <glim/util/console_colors.hpp>
#include <glim/common/imu_integration.hpp>
#include <glim/common/cloud_deskewing.hpp>
#include <glim/common/cloud_covariance_estimation.hpp>
#include <glim/backend/callbacks.hpp>

namespace glim {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

using Callbacks = SubMappingCallbacks;

SubMappingParams::SubMappingParams() {
  Config config(GlobalConfig::get_config_path("config_backend"));

  enable_imu = config.param<bool>("sub_mapping", "enable_imu", true);
  enable_optimization = config.param<bool>("sub_mapping", "enable_optimization", true);

  max_num_keyframes = config.param<int>("sub_mapping", "max_num_keyframes", 15);

  keyframe_update_strategy = config.param<std::string>("sub_mapping", "keyframe_update_strategy", "OVERLAP");
  keyframe_update_interval_rot = config.param<double>("sub_mapping", "keyframe_update_interval_rot", 3.15);
  keyframe_update_interval_trans = config.param<double>("sub_mapping", "keyframe_update_interval_trans", 1.0);
  max_keyframe_overlap = config.param<double>("sub_mapping", "max_keyframe_overlap", 0.8);

  create_between_factors = config.param<bool>("sub_mapping", "create_between_factors", true);
  between_registration_type = config.param<std::string>("sub_mapping", "between_registration_type", "GICP");

  registration_error_factor_type = config.param<std::string>("sub_mapping", "registration_error_factor_type", "VGICP");
  keyframe_randomsampling_rate = config.param<double>("sub_mapping", "keyframe_randomsampling_rate", 0.1);
  keyframe_voxel_resolution = config.param<double>("sub_mapping", "keyframe_voxel_resolution", 0.5);
  keyframe_voxelmap_levels = config.param<int>("sub_mapping", "keyframe_voxelmap_levels", 3);
  keyframe_voxelmap_scaling_factor = config.param<double>("sub_mapping", "keyframe_voxelmap_scaling_factor", 2.0);

  submap_downsample_resolution = config.param<double>("sub_mapping", "submap_downsample_resolution", 0.25);
  submap_voxel_resolution = config.param<double>("sub_mapping", "submap_voxel_resolution", 0.5);

  enable_gpu = false;
  if (registration_error_factor_type.find("GPU") != std::string::npos) {
    enable_gpu = true;
  }
}

SubMappingParams::~SubMappingParams() {}

SubMapping::SubMapping(const SubMappingParams& params) : params(params) {
  submap_count = 0;
  imu_integration.reset(new IMUIntegration);
  deskewing.reset(new CloudDeskewing);
  covariance_estimation.reset(new CloudCovarianceEstimation);

  values.reset(new gtsam::Values);
  graph.reset(new gtsam::NonlinearFactorGraph);

#ifdef BUILD_GTSAM_EXT_GPU
  stream = std::make_shared<gtsam_ext::CUDAStream>();
  stream_buffer_roundrobin = std::make_shared<gtsam_ext::StreamTempBufferRoundRobin>(8);
#endif
}

SubMapping::~SubMapping() {}

void SubMapping::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Callbacks::on_insert_imu(stamp, linear_acc, angular_vel);
  if (params.enable_imu) {
    imu_integration->insert_imu(stamp, linear_acc, angular_vel);
  }
}

void SubMapping::insert_frame(const EstimationFrame::ConstPtr& odom_frame_) {
  Callbacks::on_insert_frame(odom_frame_);

  EstimationFrame::ConstPtr odom_frame = odom_frame_;
  if (params.enable_gpu && !odom_frame->frame->points_gpu) {
    EstimationFrame::Ptr frame(new EstimationFrame);
    *frame = *odom_frame;

#ifdef BUILD_GTSAM_EXT_GPU
    if (params.enable_gpu) {
      auto stream = std::static_pointer_cast<gtsam_ext::CUDAStream>(this->stream);
      auto frame_gpu = std::make_shared<gtsam_ext::FrameGPU>(*frame->frame, *stream);
      frame->frame = frame_gpu;
    }
#endif

    odom_frame = frame;
  }

  const int current = odom_frames.size();
  const int last = current - 1;
  odom_frames.push_back(odom_frame);
  values->insert(X(current), gtsam::Pose3(odom_frame->T_world_sensor().matrix()));

  if (params.enable_imu && odom_frame->frame_id != FrameID::IMU) {
    spdlog::warn("odom frames are not estimated in the IMU frame while sub_mapping requires IMU estimation");
  }

  // Fix the first frame
  if (current == 0) {
    graph->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), values->at<gtsam::Pose3>(X(0)), gtsam::noiseModel::Isotropic::Precision(6, 1e8));
  }
  // Create a relative pose factor between consecutive frames
  else if (params.create_between_factors) {
    const Eigen::Isometry3d delta = odom_frames[last]->T_world_sensor().inverse() * odom_frame->T_world_sensor();

    if (params.between_registration_type == "GICP") {
      auto factor = gtsam::make_shared<gtsam_ext::IntegratedGICPFactor>(X(last), X(current), odom_frames[last]->frame, odom_frames[current]->frame);
      auto linearized = factor->linearize(*values);
      // graph->emplace_shared<gtsam::LinearContainerFactor>(linearized, *values);

      auto H = linearized->hessianBlockDiagonal()[X(current)];
      graph->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), gtsam::Pose3(delta.matrix()), gtsam::noiseModel::Gaussian::Information(H));
    } else if (params.between_registration_type == "NONE") {
      graph->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), gtsam::Pose3(delta.matrix()), gtsam::noiseModel::Isotropic::Precision(6, 1e3));
    } else {
      spdlog::warn("unknown between registration type ({})", params.between_registration_type);
    }
  }

  // Create an IMU preintegration factor
  if (params.enable_imu) {
    const gtsam::imuBias::ConstantBias imu_bias(odom_frame->imu_bias);

    values->insert(V(current), odom_frame->v_world_imu);
    values->insert(B(current), imu_bias);

    graph->emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(current), odom_frame->v_world_imu, gtsam::noiseModel::Isotropic::Precision(3, 1e3));
    graph->emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(current), imu_bias, gtsam::noiseModel::Isotropic::Precision(6, 1e6));

    if (current != 0) {
      int num_integrated = 0;
      const int imu_read_cursor = imu_integration->integrate_imu(odom_frames[last]->stamp, odom_frames[current]->stamp, imu_bias, &num_integrated);
      imu_integration->erase_imu_data(imu_read_cursor);

      graph
        ->emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(B(last), B(current), gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Isotropic::Precision(6, 1e6));
      if (num_integrated >= 2) {
        graph->emplace_shared<gtsam::ImuFactor>(X(last), V(last), X(current), V(current), B(last), imu_integration->integrated_measurements());
      } else {
        spdlog::warn("insufficient IMU data between LiDAR frames!! (sub_mapping)");
        graph->emplace_shared<gtsam::BetweenFactor<gtsam::Vector3>>(V(last), V(current), gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Precision(3, 1.0));
      }
    }
  }

  bool insert_as_keyframe = keyframes.empty();
  if (!insert_as_keyframe) {
    // Overlap-based keyframe update
    if (params.keyframe_update_strategy == "OVERLAP") {
      const double overlap =
        gtsam_ext::overlap_auto(keyframes.back()->voxelmaps.back(), odom_frame->frame, keyframes.back()->T_world_sensor().inverse() * odom_frame->T_world_sensor());
      insert_as_keyframe = overlap < params.max_keyframe_overlap;
    }
    // Displacement-based keyframe update
    else if (params.keyframe_update_strategy == "DISPLACEMENT") {
      const Eigen::Isometry3d delta_from_keyframe = keyframes.back()->T_world_sensor().inverse() * odom_frame->T_world_sensor();
      const double delta_trans = delta_from_keyframe.translation().norm();
      const double delta_angle = Eigen::AngleAxisd(delta_from_keyframe.linear()).angle();

      insert_as_keyframe = delta_trans > params.keyframe_update_interval_trans || delta_angle > params.keyframe_update_interval_rot;
    } else {
      spdlog::warn("unknown keyframe update strategy ({})", params.keyframe_update_strategy);
    }
  }

  // Create a new keyframe
  if (insert_as_keyframe) {
    insert_keyframe(current, odom_frame);
    Callbacks::on_new_keyframe(current, keyframes.back());

    // Create registration error factors (fully connected)
    for (int i = 0; i < keyframes.size() - 1; i++) {
      if (params.registration_error_factor_type == "VGICP") {
        for (const auto& voxelmap : keyframes[i]->voxelmaps) {
          graph->emplace_shared<gtsam_ext::IntegratedVGICPFactor>(X(keyframe_indices[i]), X(current), voxelmap, keyframes.back()->frame);
        }
      }
#ifdef BUILD_GTSAM_EXT_GPU
      else if (params.registration_error_factor_type == "VGICP_GPU") {
        auto roundrobin = std::static_pointer_cast<gtsam_ext::StreamTempBufferRoundRobin>(stream_buffer_roundrobin);
        auto stream_buffer = roundrobin->get_stream_buffer();
        const auto& stream = stream_buffer.first;
        const auto& buffer = stream_buffer.second;

        for (const auto& voxelmap : keyframes[i]->voxelmaps) {
          auto factor = gtsam::make_shared<gtsam_ext::IntegratedVGICPFactorGPU>(X(keyframe_indices[i]), X(current), voxelmap, keyframes.back()->frame, stream, buffer);
          graph->add(factor);
        }
      }
#endif
      else {
        spdlog::warn("unknown registration error factor type ({})", params.registration_error_factor_type);
      }
    }
  }

  if (odom_frames.size() >= 2) {
    // Drop unnecessary points data
    // The last frame may be required to compute the relative pose factor
    odom_frames[odom_frames.size() - 2] = odom_frames[odom_frames.size() - 2]->clone_wo_points();
  }

  auto new_submap = create_submap();

  if (new_submap) {
    new_submap->id = submap_count++;
    submap_queue.push_back(new_submap);
    Callbacks::on_new_submap(new_submap);

    odom_frames.clear();
    keyframes.clear();
    keyframe_indices.clear();
    values.reset(new gtsam::Values);
    graph.reset(new gtsam::NonlinearFactorGraph);
  }
}

void SubMapping::insert_keyframe(const int current, const EstimationFrame::ConstPtr& odom_frame) {
  gtsam_ext::Frame::ConstPtr deskewed_frame = odom_frame->frame;

  // Re-perform deskewing
  if (params.enable_imu && odom_frame->raw_frame) {
    const gtsam::NavState nav_world_imu(gtsam::Pose3(odom_frame->T_world_imu.matrix()), odom_frame->v_world_imu);
    const gtsam::imuBias::ConstantBias imu_bias(odom_frame->imu_bias);

    // TODO: smoothing-based pose estimation
    std::vector<double> imu_pred_times;
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> imu_pred_poses;
    imu_integration->integrate_imu(odom_frame->raw_frame->stamp, odom_frame->raw_frame->scan_end_time, nav_world_imu, imu_bias, imu_pred_times, imu_pred_poses);

    auto deskewed =
      deskewing
        ->deskew(odom_frame->T_lidar_imu.inverse(), imu_pred_times, imu_pred_poses, odom_frame->raw_frame->stamp, odom_frame->raw_frame->times, odom_frame->raw_frame->points);

    auto frame = std::make_shared<gtsam_ext::FrameCPU>(deskewed);
    for (int i = 0; i < frame->size(); i++) {
      frame->points[i] = odom_frame->T_lidar_imu.inverse() * frame->points[i];
    }
    frame->add_covs(covariance_estimation->estimate(frame->points_storage, odom_frame->raw_frame->neighbors));

    deskewed_frame = frame;
  }

  // Random sampling for registration error factors
  gtsam_ext::Frame::Ptr subsampled_frame = gtsam_ext::random_sampling(deskewed_frame, params.keyframe_randomsampling_rate, mt);

  EstimationFrame::Ptr keyframe(new EstimationFrame);
  *keyframe = *odom_frame;

  if (params.enable_gpu) {
#ifdef BUILD_GTSAM_EXT_GPU
    auto stream = std::static_pointer_cast<gtsam_ext::CUDAStream>(this->stream);
    keyframe->frame = std::make_shared<gtsam_ext::FrameGPU>(*subsampled_frame, *stream);
    keyframe->voxelmaps.clear();

    for (int i = 0; i < params.keyframe_voxelmap_levels; i++) {
      const double resolution = params.keyframe_voxel_resolution * std::pow(params.keyframe_voxelmap_scaling_factor, i);
      auto voxelmap = std::make_shared<gtsam_ext::GaussianVoxelMapGPU>(resolution, 8192 * 2, 10, 1e-3, *stream);
      voxelmap->insert(*keyframe->frame);
      keyframe->voxelmaps.push_back(voxelmap);
    }
#else
    spdlog::warn("GPU is enabled for sub_mapping but gtsam_ext was built without CUDA!!");
#endif
  } else {
    keyframe->voxelmaps.clear();
    for (int i = 0; i < params.keyframe_voxelmap_levels; i++) {
      const double resolution = params.keyframe_voxel_resolution * std::pow(params.keyframe_voxelmap_scaling_factor, i);
      auto voxelmap = std::make_shared<gtsam_ext::GaussianVoxelMapCPU>(resolution);
      voxelmap->insert(*keyframe->frame);
      keyframe->voxelmaps.push_back(voxelmap);
    }

    keyframe->frame = subsampled_frame;
  }

  keyframes.push_back(keyframe);
  keyframe_indices.push_back(current);
}

SubMap::Ptr SubMapping::create_submap(bool force_create) const {
  if (keyframes.size() < params.max_num_keyframes && !force_create) {
    return nullptr;
  }

  // Optimization
  Callbacks::on_optimize_submap(*graph, *values);
  gtsam_ext::LevenbergMarquardtExtParams lm_params;
  lm_params.setMaxIterations(20);
  if (Callbacks::on_optimization_status) {
    lm_params.callback = [](const gtsam_ext::LevenbergMarquardtOptimizationStatus& status, const gtsam::Values& values) { Callbacks::on_optimization_status(status, values); };
  }
  gtsam_ext::LevenbergMarquardtOptimizerExt optimizer(*graph, *values, lm_params);
  if (params.enable_optimization) {
    try {
      gtsam::Values optimized = optimizer.optimize();
      *values = optimized;
    } catch (std::exception& e) {
      spdlog::error("an exception was caught during sub map optimization");
      spdlog::error(e.what());
    }
  }

  // Create a submap by merging optimized frames
  SubMap::Ptr submap(new SubMap);
  submap->id = 0;

  const int center = odom_frames.size() / 2;
  submap->T_world_origin = Eigen::Isometry3d(values->at<gtsam::Pose3>(X(center)).matrix());
  submap->T_origin_endpoint_L = submap->T_world_origin.inverse() * Eigen::Isometry3d(values->at<gtsam::Pose3>(X(0)).matrix());
  submap->T_origin_endpoint_R = submap->T_world_origin.inverse() * Eigen::Isometry3d(values->at<gtsam::Pose3>(X(odom_frames.size() - 1)).matrix());

  submap->odom_frames = odom_frames;
  submap->frames.resize(odom_frames.size());
  for (int i = 0; i < odom_frames.size(); i++) {
    EstimationFrame::Ptr frame(new EstimationFrame);
    *frame = *odom_frames[i];

    const Eigen::Isometry3d T_world_sensor(values->at<gtsam::Pose3>(X(i)).matrix());
    frame->set_T_world_sensor(odom_frames[i]->frame_id, T_world_sensor);

    if (params.enable_imu) {
      frame->v_world_imu = values->at<gtsam::Vector3>(V(i));
      frame->imu_bias = values->at<gtsam::imuBias::ConstantBias>(B(i)).vector();
    }

    submap->frames[i] = frame;
  }

  std::vector<gtsam_ext::Frame::ConstPtr> keyframes_to_merge(keyframes.size());
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses_to_merge(keyframes.size());
  for (int i = 0; i < keyframes.size(); i++) {
    keyframes_to_merge[i] = keyframes[i]->frame;
    poses_to_merge[i] = submap->T_world_origin.inverse() * Eigen::Isometry3d(values->at<gtsam::Pose3>(X(keyframe_indices[i])).matrix());
  }

  // TODO: improve merging process
#ifdef BUILD_GTSAM_EXT_GPU
  if (params.enable_gpu) {
    // submap->frame = gtsam_ext::merge_frames_gpu(poses_to_merge, keyframes_to_merge, submap_downsample_resolution);
  }
#endif

  if (submap->frame == nullptr) {
    submap->frame = gtsam_ext::merge_frames_auto(poses_to_merge, keyframes_to_merge, params.submap_downsample_resolution);
  }

  return submap;
}

std::vector<SubMap::Ptr> SubMapping::get_submaps() {
  std::vector<SubMap::Ptr> submaps;
  submap_queue.swap(submaps);
  return submaps;
}

std::vector<SubMap::Ptr> SubMapping::submit_end_of_sequence() {
  std::vector<SubMap::Ptr> submaps;
  if (!odom_frames.empty()) {
    auto new_submap = create_submap(true);

    if (new_submap) {
      new_submap->id = submap_count++;
      submaps.push_back(new_submap);
    }
  }

  return submaps;
}

}  // namespace glim
