#pragma once

#include <random>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/preprocess/preprocessed_frame.hpp>

namespace glim {

/**
 * @brief Point cloud preprocessing parameters
 */
struct CloudPreprocessorParams {
public:
  CloudPreprocessorParams();
  ~CloudPreprocessorParams();

public:
  Eigen::Isometry3d T_lidar_offset;  ///< Transformation between the base frame and LiDAR

  bool use_random_grid_downsampling;  ///< If true, use random grid downsampling, otherwise, use the conventional voxel grid
  double distance_near_thresh;        ///< Minimum distance threshold
  double distance_far_thresh;         ///< Maximum distance threshold
  double downsample_resolution;       ///< Downsampling resolution
  int downsample_target;              ///< Target number of points for downsampling
  double downsample_rate;             ///< Downsamping rate (used for random grid downsampling)
  int k_correspondences;              ///< Number of neighboring points

  int num_threads;  ///< Number of threads
};

/**
 * @brief Point cloud preprocessor
 *
 */
class CloudPreprocessor {
public:
  using Points = std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor
   */
  CloudPreprocessor(const CloudPreprocessorParams& params = CloudPreprocessorParams());
  virtual ~CloudPreprocessor();

  /**
   * @brief Preprocess a raw point cloud
   *
   * @param raw_points  Raw points
   * @return Preprocessed points
   */
  virtual PreprocessedFrame::Ptr preprocess(const RawPoints::ConstPtr& raw_points);

private:
  std::vector<int> find_neighbors(const Points& points, int k) const;

private:
  using Params = CloudPreprocessorParams;
  Params params;

  mutable std::mt19937 mt;
};

}  // namespace glim