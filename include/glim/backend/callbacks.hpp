#pragma once

#include <opencv2/opencv.hpp>
#include <glim/util/callback_slot.hpp>
#include <glim/backend/sub_map.hpp>
#include <glim/frontend/estimation_frame.hpp>
#include <gtsam_ext/optimizers/levenberg_marquardt_optimization_status.hpp>

namespace glim {

struct SubMappingCallbacks {
  static CallbackSlot<void(const double, const cv::Mat&)> on_insert_image;
  static CallbackSlot<void(const double, const Eigen::Vector3d&, const Eigen::Vector3d&)> on_insert_imu;
  static CallbackSlot<void(const EstimationFrame::ConstPtr& frame)> on_insert_frame;

  static CallbackSlot<void(int id, const EstimationFrame::ConstPtr&)> on_new_keyframe;

  static CallbackSlot<void(const SubMap::ConstPtr&)> on_new_submap;

};
}