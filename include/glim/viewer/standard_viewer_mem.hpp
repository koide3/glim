#pragma once

#include <vector>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <glim/mapping/sub_map.hpp>

namespace glim {

struct SubMapMemoryStats {
public:
  SubMapMemoryStats();
  SubMapMemoryStats(const SubMap& submap);
  ~SubMapMemoryStats();

public:
  size_t id;  ///< Submap ID

  size_t frame_cpu_bytes;     ///< CPU memory usage
  size_t voxelmap_cpu_bytes;  ///< CPU memory usage for voxel map
  size_t frame_gpu_bytes;     ///< GPU memory usage
  size_t voxelmap_gpu_bytes;  ///< GPU memory usage for voxel map

  size_t odom_cpu_bytes;  ///< Total CPU memory usage for all frames
  size_t odom_gpu_bytes;  ///< Total GPU memory usage for all frames

  size_t num_custom_data;  ///< Number of custom data entries in the submap
};

struct FactorMemoryStats {
public:
  FactorMemoryStats();
  FactorMemoryStats(const gtsam::NonlinearFactor::shared_ptr& factor);
  ~FactorMemoryStats();

public:
  size_t cpu_bytes;
  size_t gpu_bytes;
};

}  // namespace glim