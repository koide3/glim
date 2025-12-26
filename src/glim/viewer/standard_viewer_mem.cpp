#include <glim/viewer/standard_viewer_mem.hpp>

#include <gtsam_points/config.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>

#ifdef GTSAM_POINTS_USE_CUDA
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#endif

namespace glim {

size_t get_mem_stats_cpu(const PreprocessedFrame& frame) {
  size_t bytes = 0;

  bytes += sizeof(double) * frame.size();                   // times
  bytes += sizeof(double) * frame.size();                   // intensities
  bytes += sizeof(Eigen::Vector4d) * frame.size();          // points
  bytes += sizeof(int) * frame.k_neighbors * frame.size();  // neighbors

  if (frame.raw_points) {
    bytes += sizeof(double) * frame.raw_points->size();           // raw points times
    bytes += sizeof(double) * frame.raw_points->size();           // raw points intensities
    bytes += sizeof(Eigen::Vector4d) * frame.raw_points->size();  // raw points points
    bytes += sizeof(Eigen::Vector4d) * frame.raw_points->size();  // raw points colors
    bytes += sizeof(uint32_t) * frame.raw_points->size();         // raw points rings
  }

  return bytes;
}

size_t get_mem_stats_cpu(const gtsam_points::PointCloud& points) {
  size_t bytes = 0;

  bytes += points.has_times() ? sizeof(double) * points.size() : 0;
  bytes += points.has_points() ? sizeof(Eigen::Vector4d) * points.size() : 0;
  bytes += points.has_normals() ? sizeof(Eigen::Vector4d) * points.size() : 0;
  bytes += points.has_covs() ? sizeof(Eigen::Matrix4d) * points.size() : 0;
  bytes += points.has_intensities() ? sizeof(double) * points.size() : 0;
  for (const auto& aux : points.aux_attributes) {
    bytes += aux.second.first * points.size();
  }

  return bytes;
}

size_t get_mem_stats_gpu(const gtsam_points::PointCloud& points) {
  size_t bytes = 0;

  bytes += points.has_times() ? sizeof(float) * points.size() : 0;
  bytes += points.has_points() ? sizeof(Eigen::Vector3f) * points.size() : 0;
  bytes += points.has_normals() ? sizeof(Eigen::Vector3f) * points.size() : 0;
  bytes += points.has_covs() ? sizeof(Eigen::Matrix3f) * points.size() : 0;
  bytes += points.has_intensities() ? sizeof(float) * points.size() : 0;

  return bytes;
}

size_t get_net_mem_stats_gpu(const gtsam_points::PointCloud& points) {
  size_t bytes = 0;

  bytes += points.has_times_gpu() ? sizeof(float) * points.size() : 0;
  bytes += points.has_points_gpu() ? sizeof(Eigen::Vector3f) * points.size() : 0;
  bytes += points.has_normals_gpu() ? sizeof(Eigen::Vector3f) * points.size() : 0;
  bytes += points.has_covs_gpu() ? sizeof(Eigen::Matrix3f) * points.size() : 0;
  bytes += points.has_intensities_gpu() ? sizeof(float) * points.size() : 0;

  return bytes;
}

size_t get_mem_stats_cpu(const gtsam_points::GaussianVoxelMap* voxelmap) {
  const auto v = dynamic_cast<const gtsam_points::GaussianVoxelMapCPU*>(voxelmap);
  if (!v) {
    return 0;
  }

  return v->num_voxels() * (sizeof(gtsam_points::VoxelInfo) + sizeof(gtsam_points::GaussianVoxel));
}

size_t get_mem_stats_gpu(const gtsam_points::GaussianVoxelMap* voxelmap) {
#ifdef GTSAM_POINTS_USE_CUDA
  const auto v = dynamic_cast<const gtsam_points::GaussianVoxelMapGPU*>(voxelmap);
  if (!v) {
    return 0;
  }
  constexpr size_t voxel_size = sizeof(int) + sizeof(Eigen::Vector3f) + sizeof(Eigen::Matrix3f);
  return v->voxelmap_info.num_voxels * voxel_size + v->voxelmap_info.num_buckets * sizeof(gtsam_points::VoxelBucket);
#else
  return 0;  // GPU voxel maps are not supported without CUDA
#endif
}

size_t get_net_mem_stats_gpu(const gtsam_points::GaussianVoxelMap* voxelmap) {
#ifdef GTSAM_POINTS_USE_CUDA
  const auto v = dynamic_cast<const gtsam_points::GaussianVoxelMapGPU*>(voxelmap);
  if (!v || !v->buckets) {
    return 0;
  }
  constexpr size_t voxel_size = sizeof(int) + sizeof(Eigen::Vector3f) + sizeof(Eigen::Matrix3f);
  return v->voxelmap_info.num_voxels * voxel_size + v->voxelmap_info.num_buckets * sizeof(gtsam_points::VoxelBucket);
#else
  return 0;  // GPU voxel maps are not supported without CUDA
#endif
}

size_t get_mem_stats_cpu(const EstimationFrame& frame) {
  size_t bytes = 0;

  bytes += sizeof(long) + sizeof(double);
  bytes += sizeof(Eigen::Isometry3d) * 3;                          // T_lidar_imu, T_world_lidar, T_world_imu
  bytes += sizeof(Eigen::Vector3d);                                // v_world_imu
  bytes += sizeof(Eigen::Matrix<double, 6, 1>);                    // imu_bias
  bytes += frame.imu_rate_trajectory.size() * sizeof(double) * 8;  // IMU-rate trajectory
  bytes += sizeof(FrameID);                                        // frame_id

  bytes += frame.raw_frame ? get_mem_stats_cpu(*frame.raw_frame) : 0;
  bytes += frame.frame ? get_mem_stats_cpu(*frame.frame) : 0;

  for (const auto& voxelmap : frame.voxelmaps) {
    bytes += get_mem_stats_cpu(voxelmap.get());
  }

  return bytes;
}

size_t get_mem_stats_gpu(const EstimationFrame& frame) {
  size_t bytes = 0;

  bytes += frame.frame ? get_mem_stats_gpu(*frame.frame) : 0;
  for (const auto& voxelmap : frame.voxelmaps) {
    bytes += get_mem_stats_gpu(voxelmap.get());
  }

  return bytes;
}

size_t get_net_mem_stats_gpu(const EstimationFrame& frame) {
  size_t bytes = 0;

  bytes += frame.frame ? get_net_mem_stats_gpu(*frame.frame) : 0;
  for (const auto& voxelmap : frame.voxelmaps) {
    bytes += get_net_mem_stats_gpu(voxelmap.get());
  }

  return bytes;
}

SubMapMemoryStats::SubMapMemoryStats()
: id(0),
  frame_cpu_bytes(0),
  voxelmap_cpu_bytes(0),
  frame_gpu_bytes(0),
  voxelmap_gpu_bytes(0),
  frame_gpu_net_bytes(0),
  voxelmap_gpu_net_bytes(0),
  odom_cpu_bytes(0),
  odom_gpu_bytes(0),
  odom_gpu_net_bytes(0),
  num_custom_data(0) {}

SubMapMemoryStats::SubMapMemoryStats(const SubMap& submap) : SubMapMemoryStats() {
  id = submap.id;
  frame_cpu_bytes = submap.frame ? get_mem_stats_cpu(*submap.frame) : 0;
  frame_gpu_bytes = submap.frame ? get_mem_stats_gpu(*submap.frame) : 0;
  frame_gpu_net_bytes = submap.frame ? get_net_mem_stats_gpu(*submap.frame) : 0;
  for (const auto& voxelmap : submap.voxelmaps) {
    voxelmap_cpu_bytes += get_mem_stats_cpu(voxelmap.get());
    voxelmap_gpu_bytes += get_mem_stats_gpu(voxelmap.get());
    voxelmap_gpu_net_bytes += get_net_mem_stats_gpu(voxelmap.get());
  }

  for (const auto& frame : submap.frames) {
    odom_cpu_bytes += frame ? get_mem_stats_cpu(*frame) : 0;
    odom_gpu_bytes += frame ? get_mem_stats_gpu(*frame) : 0;
    odom_gpu_net_bytes += frame ? get_net_mem_stats_gpu(*frame) : 0;
  }

  for (const auto& odom : submap.odom_frames) {
    odom_cpu_bytes += odom ? get_mem_stats_cpu(*odom) : 0;
    odom_gpu_bytes += odom ? get_mem_stats_gpu(*odom) : 0;
    odom_gpu_net_bytes += odom ? get_net_mem_stats_gpu(*odom) : 0;
  }

  num_custom_data = submap.custom_data.size();
}

SubMapMemoryStats::~SubMapMemoryStats() {}

FactorMemoryStats::FactorMemoryStats() : cpu_bytes(0), gpu_bytes(0) {}

FactorMemoryStats::FactorMemoryStats(const gtsam::NonlinearFactor::shared_ptr& factor) : FactorMemoryStats() {
  auto mcf = dynamic_cast<gtsam_points::IntegratedMatchingCostFactor*>(factor.get());
  if (mcf) {
    cpu_bytes = mcf->memory_usage();
    return;
  }

#ifdef GTSAM_POINTS_USE_CUDA
  auto vgicp_gpu = dynamic_cast<gtsam_points::IntegratedVGICPFactorGPU*>(factor.get());
  if (vgicp_gpu) {
    cpu_bytes = vgicp_gpu->memory_usage();
    gpu_bytes = vgicp_gpu->memory_usage_gpu();
    return;
  }
#endif
}

FactorMemoryStats::~FactorMemoryStats() {}

}  // namespace glim
