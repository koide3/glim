#include <glim/backend/sub_mapping_ct.hpp>

#include <random>
#include <boost/iterator/counting_iterator.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <gtsam_ext/types/frame_cpu.hpp>
#include <gtsam_ext/types/voxelized_frame_cpu.hpp>
#include <gtsam_ext/factors/integrated_gicp_factor.hpp>
#include <gtsam_ext/factors/integrated_vgicp_factor.hpp>
#include <gtsam_ext/optimizers/levenberg_marquardt_ext.hpp>

#include <glim/util/config.hpp>
#include <glim/util/easy_profiler.hpp>
#include <glim/backend/callbacks.hpp>

#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace glim {

using Callbacks = SubMappingCallbacks;

using gtsam::symbol_shorthand::X;

SubMappingCT::SubMappingCT() {
  Config config(GlobalConfig::get_config_path("config_backend_ct"));
  enable_optimization = config.param<bool>("sub_mapping_ct", "enable_optimization", true);
  max_num_frames = config.param<int>("sub_mapping_ct", "max_num_frames", 20);
  keyframe_update_interval_rot = config.param<double>("sub_mapping_ct", "keyframe_update_interval_rot", 1.0);
  keyframe_update_interval_trans = config.param<double>("sub_mapping_ct", "keyframe_update_interval_trans", 3.15);
  keyframe_randomsampling_rate = config.param<double>("sub_mapping_ct", "keyframe_randomsampling_rate", 0.1);
  keyframe_voxel_resolution = config.param<double>("sub_mapping_ct", "keyframe_voxel_resolution", 0.5);
  submap_downsample_resolution = config.param<double>("sub_mapping_ct", "submap_downsample_resolution", 0.1);
  submap_voxel_resolution = config.param<double>("sub_mapping_ct", "submap_voxel_resolution", 0.5);

  submap_count = 0;

  values.reset(new gtsam::Values);
  graph.reset(new gtsam::NonlinearFactorGraph);
}

SubMappingCT::~SubMappingCT() {}

void SubMappingCT::insert_frame(const EstimationFrame::ConstPtr& odom_frame) {
  Callbacks::on_insert_frame(odom_frame);

  const int current = odom_frames.size();
  const int last = current - 1;
  odom_frames.push_back(odom_frame);

  values->insert(X(current), gtsam::Pose3(odom_frame->T_world_lidar.matrix()));

  if (current == 0) {
    graph->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), gtsam::Pose3(odom_frame->T_world_lidar.matrix()), gtsam::noiseModel::Isotropic::Precision(6, 1e6));
  } else {
    auto factor = gtsam::make_shared<gtsam_ext::IntegratedGICPFactor>(X(last), X(current), odom_frames[last]->frame, odom_frames[current]->frame);
    auto linearized = factor->linearize(*values);
    auto H = linearized->hessianBlockDiagonal()[X(current)];

    // const auto noise = gtsam::Pose3::Expmap(gtsam::Vector6::Random() * 0.1);
    // values->update<gtsam::Pose3>(X(current), values->at<gtsam::Pose3>(X(current)) * noise);

    const Eigen::Isometry3d delta = odom_frames[last]->T_world_lidar.inverse() * odom_frame->T_world_lidar;
    graph->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), gtsam::Pose3(delta.matrix()), gtsam::noiseModel::Gaussian::Information(H));
  }

  bool insert_as_keyframe = keyframes.empty();
  if (!insert_as_keyframe) {
    const Eigen::Isometry3d delta_from_keyframe = keyframes.back()->T_world_lidar.inverse() * odom_frame->T_world_lidar;
    const double delta_trans = delta_from_keyframe.translation().norm();
    const double delta_angle = Eigen::AngleAxisd(delta_from_keyframe.linear()).angle();

    insert_as_keyframe = delta_trans > keyframe_update_interval_trans || delta_angle > keyframe_update_interval_rot;
  }

  if (insert_as_keyframe) {
    insert_keyframe(current, odom_frame);
    Callbacks::on_new_keyframe(current, keyframes.back());

    for (int i = 0; i < keyframes.size(); i++) {
      graph->emplace_shared<gtsam_ext::IntegratedVGICPFactor>(X(keyframe_indices[i]), X(current), voxelized_keyframes[i], keyframes.back()->frame);
    }
  }

  auto new_submap = create_submap();

  if (new_submap) {
    new_submap->id = submap_count++;
    submap_queue.push_back(new_submap);
    Callbacks::on_new_submap(new_submap);

    odom_frames.clear();
    keyframes.clear();
    voxelized_keyframes.clear();
    keyframe_indices.clear();
    values.reset(new gtsam::Values);
    graph.reset(new gtsam::NonlinearFactorGraph);
  }
}

void SubMappingCT::insert_keyframe(const int current, const EstimationFrame::ConstPtr& odom_frame) {
  auto keyframe_voxels = std::make_shared<gtsam_ext::GaussianVoxelMapCPU>(keyframe_voxel_resolution);
  keyframe_voxels->create_voxelmap(*odom_frame->frame);

  EstimationFrame::Ptr keyframe(new EstimationFrame);
  *keyframe = *odom_frame;
  keyframe->frame = gtsam_ext::random_sampling(odom_frame->frame, keyframe_randomsampling_rate, mt);

  keyframes.push_back(keyframe);
  voxelized_keyframes.push_back(keyframe_voxels);
  keyframe_indices.push_back(current);
}

SubMap::Ptr SubMappingCT::create_submap(bool force_create) const {
  if (keyframes.size() < max_num_frames && !force_create) {
    return nullptr;
  }

  Callbacks::on_optimize_submap(*graph, *values);
  gtsam_ext::LevenbergMarquardtExtParams lm_params;
  lm_params.setMaxIterations(20);
  if (Callbacks::on_optimization_status) {
    lm_params.callback = [](const gtsam_ext::LevenbergMarquardtOptimizationStatus& status, const gtsam::Values& values) { Callbacks::on_optimization_status(status, values); };
  }
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

    frame->v_world_imu.setZero();
    frame->imu_bias.setZero();

    submap->frames[i] = frame;
  }

  std::vector<gtsam_ext::Frame::ConstPtr> keyframes_to_merge(keyframes.size());
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses_to_merge(keyframes.size());
  for (int i = 0; i < keyframes.size(); i++) {
    keyframes_to_merge[i] = odom_frames[keyframe_indices[i]]->frame;
    poses_to_merge[i] = submap->T_world_origin.inverse() * Eigen::Isometry3d(values->at<gtsam::Pose3>(X(keyframe_indices[i])).matrix());
  }

  auto merged = gtsam_ext::merge_voxelized_frames(poses_to_merge, keyframes_to_merge, submap_downsample_resolution, submap_voxel_resolution);
  submap->frame = merged;

  return submap;
}

std::vector<SubMap::Ptr> SubMappingCT::get_submaps() {
  std::vector<SubMap::Ptr> submaps;
  submap_queue.swap(submaps);
  return submaps;
}

std::vector<SubMap::Ptr> SubMappingCT::submit_end_of_sequence() {
  std::vector<SubMap::Ptr> submaps;
  if (!odom_frames.empty()) {
    submaps.push_back(create_submap(true));
  }

  return submaps;
}

}  // namespace glim