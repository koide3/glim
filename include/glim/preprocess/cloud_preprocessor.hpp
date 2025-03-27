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
  double distance_near_thresh;        ///< Minimum distance threshold
  double distance_far_thresh;         ///< Maximum distance threshold
  bool global_shutter;                ///< Assume all points in a scan are takes at the same moment and replace per-point timestamps with zero (disable deskewing)
  bool use_random_grid_downsampling;  ///< If true, use random grid downsampling, otherwise, use the conventional voxel grid
  double downsample_resolution;       ///< Downsampling resolution
  int downsample_target;              ///< Target number of points for downsampling
  double downsample_rate;             ///< Downsamping rate (used for random grid downsampling)
  bool enable_outlier_removal;        ///< If true, apply statistical outlier removal
  int outlier_removal_k;              ///< Number of neighbors used for outlier removal
  double outlier_std_mul_factor;      ///< Statistical outlier removal std dev threshold multiplication factor
  bool enable_cropbox_filter;         ///< If true, filter points out points within box
  std::string crop_bbox_frame;        ///< Bounding box reference frame
  Eigen::Vector3d crop_bbox_min;      ///< Bounding box min point
  Eigen::Vector3d crop_bbox_max;      ///< Bounding box max point
  Eigen::Isometry3d T_imu_lidar;      ///< LiDAR-IMU transformation when cropbox is defined in IMU frame
  int k_correspondences;              ///< Number of neighboring points

  int num_threads;                    ///< Number of threads
};

/**
 * @brief Point cloud preprocessor
 *
 */
class CloudPreprocessor {
public:
  using Points = std::vector<Eigen::Vector4d>;
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
  PreprocessedFrame::Ptr preprocess_impl(const RawPoints::ConstPtr& raw_points);
  std::vector<int> find_neighbors(const Eigen::Vector4d* points, const int num_points, const int k) const;

private:
  using Params = CloudPreprocessorParams;
  Params params;

  mutable std::mt19937 mt;

  std::shared_ptr<void> tbb_task_arena;
};

}  // namespace glim