#pragma once

#include <memory>
#include <random>
#include <glim/backend/sub_mapping_base.hpp>

namespace gtsam {
class Values;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace gtsam_ext {
class GaussianVoxelMapCPU;
}

namespace glim {

class SubMappingCT : public SubMappingBase {
public:
  SubMappingCT();
  virtual ~SubMappingCT();

  virtual void insert_frame(const EstimationFrame::ConstPtr& odom_frame) override;

  virtual std::vector<SubMap::Ptr> get_submaps() override;

  virtual std::vector<SubMap::Ptr> submit_end_of_sequence() override;

private:
  void insert_keyframe(const int current, const EstimationFrame::ConstPtr& odom_frame);

  SubMap::Ptr create_submap(bool force_create = false) const;

private:
  bool enable_optimization;
  int max_num_frames;
  double keyframe_update_interval_rot;
  double keyframe_update_interval_trans;
  double keyframe_randomsampling_rate;
  double keyframe_voxel_resolution;
  double submap_downsample_resolution;
  double submap_voxel_resolution;

  std::mt19937 mt;
  int submap_count;

  std::vector<EstimationFrame::ConstPtr> odom_frames;

  std::vector<int> keyframe_indices;
  std::vector<EstimationFrame::ConstPtr> keyframes;
  std::vector<std::shared_ptr<gtsam_ext::GaussianVoxelMapCPU>> voxelized_keyframes;

  std::unique_ptr<gtsam::Values> values;
  std::unique_ptr<gtsam::NonlinearFactorGraph> graph;

  std::vector<SubMap::Ptr> submap_queue;
};
};  // namespace glim