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
class IntegratedCT_GICPFactor;
}

namespace glim {

class CloudCovarianceEstimation;

/**
 * @brief LiDAR-only odometry estimation based on CT-GICP scan-to-model matching
 */
class OdometryEstimationCT : public OdometryEstimationBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationCT();
  virtual ~OdometryEstimationCT() override;

  virtual EstimationFrame::ConstPtr insert_frame(const PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) override;

private:
  struct TargetMap;
  std::shared_ptr<TargetMap> create_target_map(const std::vector<EstimationFrame::ConstPtr>& keyframes);

private:
  int num_threads;
  double max_correspondence_distance;
  int max_num_keyframes;
  double keyframe_update_interval_rot;
  double keyframe_update_interval_trans;

  double stiffness_scale_first;
  double stiffness_scale_second;
  int lm_max_iterations_first;
  int lm_max_iterations_second;

  std::unique_ptr<CloudCovarianceEstimation> covariance_estimation;

  int frame_count;
  EstimationFrame::Ptr last_frame;
  std::deque<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>> v_last_current_history;

  std::deque<EstimationFrame::Ptr> keyframes;

  std::shared_ptr<TargetMap> target_map;
  std::future<std::shared_ptr<TargetMap>> async_target;
};

}  // namespace glim