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
}

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

  double stiffness_scale_first;
  double stiffness_scale_second;
  int lm_max_iterations_first;
  int lm_max_iterations_second;
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

  EstimationFrame::Ptr last_frame;
  std::deque<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>> v_last_current_history;

  std::shared_ptr<gtsam_ext::iVox> target_ivox;
};

}  // namespace glim