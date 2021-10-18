#pragma once

#include <opencv2/opencv.hpp>
#include <glim/util/callback_slot.hpp>
#include <glim/frontend/estimation_frame.hpp>

#include <gtsam_ext/optimizers/incremental_fixed_lag_smoother_ext.hpp>

namespace glim {

struct OdometryEstimationCallbacks {
  static CallbackSlot<void(const double, const cv::Mat&)> on_insert_image;
  static CallbackSlot<void(const double, const Eigen::Vector3d&, const Eigen::Vector3d&)> on_insert_imu;
  static CallbackSlot<void(const PreprocessedFrame::Ptr& frame)> on_insert_frame;

  static CallbackSlot<void(const EstimationFrame::ConstPtr&)> on_new_frame;

  static CallbackSlot<void(const std::vector<EstimationFrame::ConstPtr>&)> on_update_frames;
  static CallbackSlot<void(const std::vector<EstimationFrame::ConstPtr>&)> on_update_keyframes;
  static CallbackSlot<void(const std::vector<EstimationFrame::ConstPtr>&)> on_marginalized_frames;

  static CallbackSlot<void(gtsam_ext::IncrementalFixedLagSmootherExt&, gtsam::NonlinearFactorGraph&, gtsam::Values&)> on_smoother_update;

  static CallbackSlot<void()> on_smoother_corruption;
};

}  // namespace glim