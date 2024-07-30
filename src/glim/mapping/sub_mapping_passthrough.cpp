#include <glim/mapping/sub_mapping_passthrough.hpp>

#include <spdlog/spdlog.h>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/util/easy_profiler.hpp>

#include <glim/util/config.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/common/cloud_covariance_estimation.hpp>
#include <glim/common/cloud_deskewing.hpp>

namespace glim {

using Callbacks = SubMappingCallbacks;

SubMappingPassthroughParams::SubMappingPassthroughParams() {
  Config config(GlobalConfig::get_config_path("config_sub_mapping"));

  keyframe_update_interval_rot = config.param<double>("sub_mapping", "keyframe_update_interval_rot", 0.01);
  keyframe_update_interval_trans = config.param<double>("sub_mapping", "keyframe_update_interval_trans", 0.1);

  max_num_keyframes = config.param<int>("sub_mapping", "max_num_keyframes", 50);
  max_num_keyframes = max_num_keyframes < 0 ? std::numeric_limits<int>::max() : max_num_keyframes;

  max_num_voxels = config.param<int>("sub_mapping", "max_num_voxels", 50000);
  max_num_voxels = max_num_voxels < 0 ? std::numeric_limits<int>::max() : max_num_voxels;

  adaptive_max_num_voxels = config.param<double>("sub_mapping", "adaptive_max_num_voxels", 0.5);
  adaptive_max_num_voxels = adaptive_max_num_voxels < 0 ? std::numeric_limits<double>::max() : adaptive_max_num_voxels;

  submap_target_num_points = config.param<int>("sub_mapping", "submap_target_num_points", 40000);
  submap_voxel_resolution = config.param<double>("sub_mapping", "submap_voxel_resolution", 0.5);
  min_dist_in_voxel = config.param<double>("sub_mapping", "min_dist_in_voxel", 0.1);
  max_num_points_in_voxel = config.param<int>("sub_mapping", "max_num_points_in_voxel", 100);
}

SubMappingPassthroughParams::~SubMappingPassthroughParams() {}

SubMappingPassthrough::SubMappingPassthrough(const SubMappingPassthroughParams& params) : params(params) {
  submap_count = 0;

  voxelmap.reset(new VoxelMap(params.submap_voxel_resolution));
  voxelmap->set_lru_horizon(std::numeric_limits<int>::max());
  voxelmap->set_lru_clear_cycle(std::numeric_limits<int>::max());
  voxelmap->set_neighbor_voxel_mode(1);
  voxelmap->voxel_insertion_setting().set_max_num_points_in_cell(params.max_num_points_in_voxel);
  voxelmap->voxel_insertion_setting().set_min_dist_in_cell(params.min_dist_in_voxel);
}

SubMappingPassthrough::~SubMappingPassthrough() {}

void SubMappingPassthrough::insert_frame(const EstimationFrame::ConstPtr& odom_frame) {
  logger->trace("insert_frame frame_id={} stamp={}", odom_frame->id, odom_frame->stamp);
  Callbacks::on_insert_frame(odom_frame);

  const int current = odom_frames.size();
  odom_frames.emplace_back(odom_frame->clone_wo_points());

  // Check if the current frame should be inserted as a keyframe
  bool insert_as_keyframe = true;
  if (!keyframes.empty()) {
    const Eigen::Isometry3d T_last_current = keyframes.back()->T_world_sensor().inverse() * odom_frame->T_world_sensor();
    const double dt = T_last_current.translation().norm();
    const double dr = Eigen::AngleAxisd(T_last_current.linear()).angle();
    insert_as_keyframe = dt > params.keyframe_update_interval_trans || dr > params.keyframe_update_interval_rot;
    logger->debug("dt={} dr={} keyframe={}", dt, dr, insert_as_keyframe);
  }

  if (insert_as_keyframe) {
    Callbacks::on_new_keyframe(current, odom_frame);
    keyframes.emplace_back(odom_frame->clone_wo_points());

    // Insert the current frame into the voxel map
    // TODO: May need to consider accumulating points in a local frame to avoid voxel coord overflow
    auto transformed = gtsam_points::transform(odom_frame->frame, odom_frame->T_world_sensor());
    voxelmap->insert(*transformed);
    num_voxels_history.emplace_back(voxelmap->num_voxels());
    logger->debug("num_voxels={}", voxelmap->num_voxels());
  }

  auto new_submap = create_submap();

  if (new_submap) {
    // A new submap is created
    new_submap->id = submap_count++;
    submap_queue.push_back(new_submap);
    Callbacks::on_new_submap(new_submap);

    odom_frames.clear();
    keyframes.clear();
    voxelmap->clear();
    num_voxels_history.clear();
  }
}

std::vector<SubMap::Ptr> SubMappingPassthrough::get_submaps() {
  std::vector<SubMap::Ptr> submaps;
  submap_queue.swap(submaps);
  return submaps;
}

std::vector<SubMap::Ptr> SubMappingPassthrough::submit_end_of_sequence() {
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

SubMap::Ptr SubMappingPassthrough::create_submap(bool force_create) const {
  // Check for adaptive maximum number of voxel
  const auto check_adaptive_num_voxels = [&] {
    if (num_voxels_history.size() < 3) {
      return true;
    }

    const size_t init_num_voxels = num_voxels_history[2];
    return voxelmap->num_voxels() < init_num_voxels * params.adaptive_max_num_voxels;
  };

  // Check if there are enough frames to compose a submap
  if (!force_create && keyframes.size() < params.max_num_keyframes && voxelmap->num_voxels() < params.max_num_voxels && check_adaptive_num_voxels()) {
    return nullptr;
  }

  logger->debug("create submap");

  // Create a submap
  SubMap::Ptr submap(new SubMap);
  submap->id = 0;

  const int center = odom_frames.size() / 2;
  submap->T_world_origin = odom_frames[center]->T_world_sensor();
  submap->T_origin_endpoint_L = submap->T_world_origin.inverse() * odom_frames.front()->T_world_sensor();
  submap->T_origin_endpoint_R = submap->T_world_origin.inverse() * odom_frames.back()->T_world_sensor();

  submap->odom_frames = odom_frames;
  submap->frames = odom_frames;

  auto merged = voxelmap->voxel_data();
  submap->frame = gtsam_points::transform(merged, submap->T_world_origin.inverse());

  if (params.submap_target_num_points > 0 && submap->frame->size() > params.submap_target_num_points) {
    std::mt19937 mt(submap_count * 643145 + submap->frame->size() * 4312);  // Just a random-like seed
    submap->frame = gtsam_points::random_sampling(submap->frame, static_cast<double>(params.submap_target_num_points) / submap->frame->size(), mt);
    logger->debug("|subsampled_submap|={}", submap->frame->size());
  }

  return submap;
}

}  // namespace glim