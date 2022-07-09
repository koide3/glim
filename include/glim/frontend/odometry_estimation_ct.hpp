#pragma once

#include <deque>
#include <memory>
#include <future>

#include <boost/shared_ptr.hpp>
#include <glim/frontend/odometry_estimation_base.hpp>

namespace gtsam {
class Values;
}

namespace gtsam_ext {
class iVox;
class IncrementalFixedLagSmootherExt;
class IncrementalFixedLagSmootherExtWithFallback;
}  // namespace gtsam_ext

namespace glim {

class CloudCovarianceEstimation;

struct OdometryEstimationCTParams {
public:
  OdometryEstimationCTParams();
  ~OdometryEstimationCTParams();

public:
  int num_threads;
  double max_correspondence_distance;

  double ivox_resolution;
  double ivox_min_points_dist;
  int ivox_lru_thresh;

  double location_consistency_inf_scale;
  double constant_velocity_inf_scale;
  int lm_max_iterations;

  // iSAM2 params
  double smoother_lag;
  bool use_isam2_dogleg;
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

  virtual EstimationFrame::ConstPtr insert_frame(const PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) override;

private:
  using Params = OdometryEstimationCTParams;
  Params params;

  std::unique_ptr<CloudCovarianceEstimation> covariance_estimation;

  int marginalized_cursor;
  std::vector<EstimationFrame::Ptr> frames;

  std::shared_ptr<gtsam_ext::iVox> target_ivox;
  EstimationFrame::ConstPtr target_ivox_frame;

  // Optimizer
  using FixedLagSmootherExt = gtsam_ext::IncrementalFixedLagSmootherExtWithFallback;
  std::unique_ptr<FixedLagSmootherExt> smoother;
};

}  // namespace glim