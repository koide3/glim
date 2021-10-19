#pragma once

#include <memory>
#include <glim/backend/sub_mapping_base.hpp>

namespace gtsam {
class Values;
class NonlinearFactorGraph;
class PreintegratedImuMeasurements;
}  // namespace gtsam

namespace gtsam_ext {
class StreamTempBufferRoundRobin;
}

namespace glim {

class IMUIntegration;
class CloudDeskewing;
class CloudCovarianceEstimation;

class SubMapping : public SubMappingBase {
public:
  SubMapping();
  virtual ~SubMapping() override;

  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;
  virtual void insert_frame(const EstimationFrame::ConstPtr& odom_frame) override;

  virtual std::vector<SubMap::Ptr> get_submaps() override;

  virtual std::vector<SubMap::Ptr> submit_end_of_sequence() override;

private:
  EstimationFrame::Ptr create_keyframe(const EstimationFrame::ConstPtr& odom_frame) const;

  SubMap::Ptr create_submap(bool force_create = false) const;

private:
  bool enable_optimization;
  int min_num_frames;
  int max_num_frames;
  double max_keyframe_overlap;
  double min_keyframe_overlap;
  double submap_downsample_resolution;
  double submap_voxel_resolution;

  int submap_count; 

  std::unique_ptr<IMUIntegration> imu_integration;
  std::unique_ptr<CloudDeskewing> deskewing;
  std::unique_ptr<CloudCovarianceEstimation> covariance_estimation;

  std::unique_ptr<gtsam_ext::StreamTempBufferRoundRobin> stream_buffer_roundrobin;

  std::vector<EstimationFrame::ConstPtr> odom_frames;

  std::vector<int> keyframe_indices;
  std::vector<EstimationFrame::Ptr> keyframes;

  std::unique_ptr<gtsam::Values> values;
  std::unique_ptr<gtsam::NonlinearFactorGraph> graph;

  std::vector<SubMap::Ptr> submap_queue;
};

}  // namespace glim
