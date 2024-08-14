#pragma once

#include <deque>
#include <memory>
#include <future>

#include <boost/shared_ptr.hpp>
#include <glim/odometry/odometry_estimation_base.hpp>

namespace gtsam {
class Values;
}

namespace gtsam_points {
struct FlatContainer;
template <typename VoxelContents>
class IncrementalVoxelMap;
using iVox = IncrementalVoxelMap<FlatContainer>;

class IncrementalCovarianceVoxelMap;
using iVoxCovarianceEstimation = IncrementalCovarianceVoxelMap;
class IncrementalFixedLagSmootherExt;
class IncrementalFixedLagSmootherExtWithFallback;
}  // namespace gtsam_points

namespace glim {

class CloudCovarianceEstimation;

/**
 * @brief Parameters for OdometryEstimationCT
 */
struct OdometryEstimationCTParams {
public:
  OdometryEstimationCTParams();
  ~OdometryEstimationCTParams();

public:
  int num_threads;  ///< Number of threads

  double ivox_resolution;       ///< iVox resolution
  double ivox_min_points_dist;  ///< Minimum distance between points in an iVox cell
  int ivox_lru_thresh;          ///< iVox LRU cache threshold

  double max_correspondence_distance;     ///< Maximum distance between corresponding points
  double location_consistency_inf_scale;  ///< Weight for location consistency constraints
  double constant_velocity_inf_scale;     ///< Weight for constant velocity constraints
  int lm_max_iterations;                  ///< Maximum number of iterations for LM optimization

  // iSAM2 params
  double smoother_lag;    ///< Fixed-lag smoothing window [sec]
  bool use_isam2_dogleg;  ///< If true, use dogleg optimizer
  double isam2_relinearize_skip;
  double isam2_relinearize_thresh;
};

/**
 * @brief LiDAR-only odometry estimation based on CT-GICP scan-to-model matching
 */
class OdometryEstimationCT : public OdometryEstimationBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationCT(const OdometryEstimationCTParams& params = OdometryEstimationCTParams());
  virtual ~OdometryEstimationCT() override;

  virtual bool requires_imu() const override { return false; }

  virtual EstimationFrame::ConstPtr insert_frame(const PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) override;

private:
  using Params = OdometryEstimationCTParams;
  Params params;

  std::unique_ptr<CloudCovarianceEstimation> covariance_estimation;

  int marginalized_cursor;
  std::vector<EstimationFrame::Ptr> frames;         ///< Estimation frames
  std::shared_ptr<gtsam_points::iVox> target_ivox;  ///< Target iVox
  EstimationFrame::ConstPtr target_ivox_frame;      ///< Target iVox points (just for visualization)

  // Optimizer
  using FixedLagSmootherExt = gtsam_points::IncrementalFixedLagSmootherExtWithFallback;
  std::unique_ptr<FixedLagSmootherExt> smoother;

  std::shared_ptr<void> tbb_task_arena;
};

}  // namespace glim