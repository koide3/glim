#pragma once

#include <opencv2/opencv.hpp>
#include <gtsam/nonlinear/Values.h>

#include <glim/util/callback_slot.hpp>
#include <glim/backend/sub_map.hpp>
#include <glim/frontend/estimation_frame.hpp>

#include <gtsam_ext/optimizers/isam2_ext.hpp>
#include <gtsam_ext/optimizers/levenberg_marquardt_optimization_status.hpp>

namespace glim {

struct SubMappingCallbacks {
  static CallbackSlot<void(const double, const cv::Mat&)> on_insert_image;
  static CallbackSlot<void(const double, const Eigen::Vector3d&, const Eigen::Vector3d&)> on_insert_imu;
  static CallbackSlot<void(const EstimationFrame::ConstPtr& frame)> on_insert_frame;

  static CallbackSlot<void(int id, const EstimationFrame::ConstPtr&)> on_new_keyframe;

  static CallbackSlot<void(const SubMap::ConstPtr&)> on_new_submap;
};

struct GlobalMappingCallbacks {
  static CallbackSlot<void(const double, const cv::Mat&)> on_insert_image;
  static CallbackSlot<void(const double, const Eigen::Vector3d&, const Eigen::Vector3d&)> on_insert_imu;
  static CallbackSlot<void(const SubMap::ConstPtr& frame)> on_insert_submap;

  static CallbackSlot<void(const std::vector<SubMap::Ptr>& submaps)> on_update_submaps;

  static CallbackSlot<void(gtsam_ext::ISAM2Ext&, gtsam::NonlinearFactorGraph&, gtsam::Values&)> on_smoother_update;
};
}