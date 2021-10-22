#pragma once

#include <glim/util/callback_slot.hpp>
#include <glim/frontend/estimation_frame.hpp>
#include <glim/backend/sub_map.hpp>

namespace cv {
class Mat;
}

namespace gtsam {
class Values;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace gtsam_ext {
class ISAM2Ext;
class ISAM2ResultExt;
class LevenbergMarquardtOptimizationStatus;
}  // namespace gtsam_ext

namespace glim {

struct SubMappingCallbacks {
  static CallbackSlot<void(const double, const cv::Mat&)> on_insert_image;
  static CallbackSlot<void(const double, const Eigen::Vector3d&, const Eigen::Vector3d&)> on_insert_imu;
  static CallbackSlot<void(const EstimationFrame::ConstPtr& frame)> on_insert_frame;

  static CallbackSlot<void(int id, const EstimationFrame::ConstPtr&)> on_new_keyframe;

  static CallbackSlot<void(gtsam::NonlinearFactorGraph&, gtsam::Values&)> on_optimize_submap;
  static CallbackSlot<void(const gtsam_ext::LevenbergMarquardtOptimizationStatus&, const gtsam::Values& values)> on_optimization_status;
  static CallbackSlot<void(const SubMap::ConstPtr&)> on_new_submap;
};

struct GlobalMappingCallbacks {
  static CallbackSlot<void(const double, const cv::Mat&)> on_insert_image;
  static CallbackSlot<void(const double, const Eigen::Vector3d&, const Eigen::Vector3d&)> on_insert_imu;
  static CallbackSlot<void(const SubMap::ConstPtr& frame)> on_insert_submap;

  static CallbackSlot<void(const std::vector<SubMap::Ptr>& submaps)> on_update_submaps;

  static CallbackSlot<void(gtsam_ext::ISAM2Ext&, gtsam::NonlinearFactorGraph&, gtsam::Values&)> on_smoother_update;
  static CallbackSlot<void(gtsam_ext::ISAM2Ext&, const gtsam_ext::ISAM2ResultExt& result)> on_smoother_update_result;
};
}