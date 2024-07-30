#pragma once

#include <deque>
#include <random>
#include <memory>
#include <glim/mapping/sub_mapping_base.hpp>
#include <gtsam_points/ann/flat_container.hpp>
#include <gtsam_points/ann/incremental_voxelmap.hpp>

namespace glim {

class CloudDeskewing;
class CloudCovarianceEstimation;
using VoxelMap = gtsam_points::IncrementalVoxelMap<gtsam_points::FlatContainer>;

/**
 * @brief SubMappingPassthrough parameters
 */
struct SubMappingPassthroughParams {
public:
  SubMappingPassthroughParams();
  ~SubMappingPassthroughParams();

public:
  double keyframe_update_interval_rot;
  double keyframe_update_interval_trans;

  int max_num_keyframes;
  int max_num_voxels;
  double adaptive_max_num_voxels;

  int submap_target_num_points;
  double submap_voxel_resolution;
  double min_dist_in_voxel;
  int max_num_points_in_voxel;
};

/**
 * @brief Sub mapping based on voxel-based simple frame merging. No optimization, no re-deskewing.
 */
class SubMappingPassthrough : public SubMappingBase {
public:
  SubMappingPassthrough(const SubMappingPassthroughParams& params = SubMappingPassthroughParams());
  virtual ~SubMappingPassthrough() override;

  virtual void insert_frame(const EstimationFrame::ConstPtr& odom_frame) override;

  virtual std::vector<SubMap::Ptr> get_submaps() override;

  virtual std::vector<SubMap::Ptr> submit_end_of_sequence() override;

private:
  SubMap::Ptr create_submap(bool force_create = false) const;

private:
  using Params = SubMappingPassthroughParams;
  Params params;

  int submap_count;

  std::vector<EstimationFrame::ConstPtr> odom_frames;
  std::vector<EstimationFrame::ConstPtr> keyframes;
  std::unique_ptr<VoxelMap> voxelmap;
  std::vector<size_t> num_voxels_history;

  std::vector<SubMap::Ptr> submap_queue;
};

}  // namespace glim
