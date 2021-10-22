#include <glim/backend/sub_mapping.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam_ext/types/voxelized_frame.hpp>
#include <gtsam_ext/types/voxelized_frame_gpu.hpp>
#include <gtsam_ext/factors/integrated_gicp_factor.hpp>
#include <gtsam_ext/factors/integrated_vgicp_factor.hpp>
#include <gtsam_ext/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_ext/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_ext/cuda/stream_temp_buffer_roundrobin.hpp>

#include <glim/util/config.hpp>
#include <glim/common/imu_integration.hpp>
#include <glim/common/cloud_deskewing.hpp>
#include <glim/common/cloud_covariance_estimation.hpp>
#include <glim/backend/callbacks.hpp>

namespace glim
{

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

using Callbacks = SubMappingCallbacks;

SubMapping::SubMapping() {
  Config config(GlobalConfig::get_config_path("config_backend"));

  enable_optimization = config.param<bool>("sub_mapping", "enable_optimization", true);
  min_num_frames = config.param<int>("sub_mapping", "min_num_frames", 5);
  max_num_frames = config.param<int>("sub_mapping", "max_num_frames", 15);
  min_keyframe_overlap = config.param<double>("sub_mapping", "min_keyframe_overlap", 0.05);
  max_keyframe_overlap = config.param<double>("sub_mapping", "max_keyframe_overlap", 0.8);
  submap_downsample_resolution = config.param<double>("sub_mapping", "submap_downsample_resolution", 0.25);
  submap_voxel_resolution = config.param<double>("sub_mapping", "submap_voxel_resolution", 0.5);

  submap_count = 0;
  imu_integration.reset(new IMUIntegration);
  deskewing.reset(new CloudDeskewing);
  covariance_estimation.reset(new CloudCovarianceEstimation);
  stream_buffer_roundrobin.reset(new gtsam_ext::StreamTempBufferRoundRobin(32));

  values.reset(new gtsam::Values);
  graph.reset(new gtsam::NonlinearFactorGraph);
}

SubMapping::~SubMapping() {}

void SubMapping::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Callbacks::on_insert_imu(stamp, linear_acc, angular_vel);
  imu_integration->insert_imu(stamp, linear_acc, angular_vel);
}

void SubMapping::insert_frame(const EstimationFrame::ConstPtr& odom_frame) {
  Callbacks::on_insert_frame(odom_frame);

  const int current = odom_frames.size();
  const int last = current - 1;
  odom_frames.push_back(odom_frame);

  const gtsam::imuBias::ConstantBias imu_bias(odom_frame->imu_bias);

  values->insert(X(current), gtsam::Pose3(odom_frame->T_world_imu.matrix()));
  values->insert(V(current), odom_frame->v_world_imu);
  values->insert(B(current), imu_bias);

  graph->emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(current), odom_frame->v_world_imu, gtsam::noiseModel::Isotropic::Precision(3, 1e3));
  graph->emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(current), imu_bias, gtsam::noiseModel::Isotropic::Precision(6, 1e6));

  if(current == 0) {
    graph->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(current), gtsam::Pose3(odom_frame->T_world_imu.matrix()), gtsam::noiseModel::Isotropic::Precision(6, 1e6));
  } else {
    int num_integrated = 0;
    const int imu_read_cursor = imu_integration->integrate_imu(odom_frames[last]->stamp, odom_frames[current]->stamp, imu_bias, &num_integrated);
    imu_integration->erase_imu_data(imu_read_cursor);

    graph->emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(B(last), B(current), gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Isotropic::Precision(6, 1e6));
    if(num_integrated) {
      graph->emplace_shared<gtsam::ImuFactor>(X(last), V(last), X(current), V(current), B(last), imu_integration->integrated_measurements());
    } else {
      std::cerr << "warning: no IMU data between LiDAR frames!! (sub_mapping)" << std::endl;
      graph->emplace_shared<gtsam::BetweenFactor<gtsam::Vector3>>(V(last), V(current), gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Precision(3, 1.0));
    }
  }

  bool insert_as_keyframe = keyframes.empty();
  if(!insert_as_keyframe) {
    std::vector<gtsam_ext::VoxelizedFrame::ConstPtr> targets(keyframes.size());
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> deltas(keyframes.size());
    for (int i = 0; i < keyframes.size(); i++) {
      targets[i] = keyframes[i]->voxelized_frame();
      deltas[i] = keyframes[i]->T_world_imu.inverse() * odom_frame->T_world_imu;
    }

    const double overlap = odom_frame->frame->overlap_gpu(targets, deltas);
    if (overlap < max_keyframe_overlap) {
      insert_as_keyframe = true;
    }
  }

  if(insert_as_keyframe) {
    keyframe_indices.push_back(current);
    keyframes.push_back(create_keyframe(odom_frame));
    Callbacks::on_new_keyframe(current, keyframes.back());

    for (int i = 0; i < keyframes.size() - 1; i++) {
      auto stream_buffer = stream_buffer_roundrobin->get_stream_buffer();
      const auto& stream = stream_buffer.first;
      const auto& buffer = stream_buffer.second;

      graph->emplace_shared<gtsam_ext::IntegratedVGICPFactorGPU>(X(keyframe_indices[i]), X(current), keyframes[i]->voxelized_frame(), keyframes.back()->frame, stream, buffer);
    }
  }

  auto new_submap = create_submap();

  if(new_submap) {
    new_submap->id = submap_count++;

    Callbacks::on_new_submap(new_submap);
    submap_queue.push_back(new_submap);

    odom_frames.clear();
    keyframes.clear();
    keyframe_indices.clear();
    values.reset(new gtsam::Values);
    graph.reset(new gtsam::NonlinearFactorGraph);
  }
}

EstimationFrame::Ptr SubMapping::create_keyframe(const EstimationFrame::ConstPtr& odom_frame) const {
  const gtsam::NavState nav_world_imu(gtsam::Pose3(odom_frame->T_world_imu.matrix()), odom_frame->v_world_imu);
  const gtsam::imuBias::ConstantBias imu_bias(odom_frame->imu_bias);

  std::vector<double> imu_pred_times;
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> imu_pred_poses;
  imu_integration->integrate_imu(odom_frame->raw_frame->stamp, odom_frame->raw_frame->scan_end_time, nav_world_imu, imu_bias, imu_pred_times, imu_pred_poses);

  auto deskewed = deskewing->deskew(odom_frame->T_lidar_imu.inverse(), imu_pred_times, imu_pred_poses, odom_frame->raw_frame->stamp, odom_frame->raw_frame->times, odom_frame->raw_frame->points);
  auto covs = covariance_estimation->estimate(deskewed, odom_frame->raw_frame->neighbors);

  EstimationFrame::Ptr keyframe(new EstimationFrame);
  keyframe->id = odom_frame->id;
  keyframe->stamp = odom_frame->stamp;

  keyframe->T_lidar_imu = odom_frame->T_lidar_imu;
  keyframe->T_world_lidar = odom_frame->T_world_lidar;
  keyframe->T_world_imu = odom_frame->T_world_imu;

  keyframe->v_world_imu = odom_frame->v_world_imu;
  keyframe->imu_bias = odom_frame->imu_bias;

  keyframe->raw_frame = odom_frame->raw_frame;

  keyframe->frame_id = odom_frame->frame_id;
  keyframe->frame = std::make_shared<gtsam_ext::VoxelizedFrameGPU>(odom_frame->voxelized_frame()->voxel_resolution(), deskewed, covs);

  return keyframe;
}

SubMap::Ptr SubMapping::create_submap(bool force_create) const {
  if (keyframes.size() < min_num_frames && !force_create) {
    return nullptr;
  }

  if (keyframes.size() < max_num_frames && !force_create) {
    const Eigen::Isometry3d delta_first_last = keyframes.front()->T_world_imu.inverse() * keyframes.back()->T_world_imu;
    const double overlap_first_last = keyframes.back()->frame->overlap_gpu(keyframes.front()->voxelized_frame(), delta_first_last);

    if(overlap_first_last > min_num_frames) {
      return nullptr;
    }
  }

  gtsam_ext::LevenbergMarquardtExtParams lm_params;
  gtsam_ext::LevenbergMarquardtOptimizerExt optimizer(*graph, *values, lm_params);
  *values = optimizer.optimize();

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
    frame->id = odom_frames[i]->id;
    frame->stamp = odom_frames[i]->stamp;

    frame->T_lidar_imu = odom_frames[i]->T_lidar_imu;
    frame->T_world_imu = Eigen::Isometry3d(values->at<gtsam::Pose3>(X(i)).matrix());
    frame->T_world_lidar = frame->T_world_imu * frame->T_lidar_imu.inverse();

    frame->v_world_imu = values->at<gtsam::Vector3>(V(i));
    frame->imu_bias = values->at<gtsam::imuBias::ConstantBias>(B(i)).vector();

    submap->frames[i] = frame;
  }

  std::vector<gtsam_ext::Frame::ConstPtr> keyframes_to_merge(keyframes.size());
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses_to_merge(keyframes.size());
  for (int i = 0; i < keyframes.size(); i++){
    keyframes_to_merge[i] = keyframes[i]->frame;
    poses_to_merge[i] = submap->T_world_origin.inverse() * Eigen::Isometry3d(values->at<gtsam::Pose3>(X(keyframe_indices[i])).matrix());
  }

  const bool allocate_cpu = keyframes.front()->frame->points != nullptr;
  auto merged = gtsam_ext::merge_voxelized_frames_gpu(poses_to_merge, keyframes_to_merge, submap_downsample_resolution, submap_voxel_resolution, allocate_cpu);
  submap->frame = merged;

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
    submaps.push_back(create_submap(true));
  }

  return submaps;
}

} // namespace glim
