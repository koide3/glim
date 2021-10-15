#pragma once

#include <glim/util/callback_slot.hpp>
#include <glim/frontend/estimation_frame.hpp>

#include <gtsam_ext/optimizers/incremental_fixed_lag_smoother_ext.hpp>

namespace glim {

struct OdometryEstimationCallbacks {
  static CallbackSlot<void()> on_smoother_corruption;
  static CallbackSlot<void(const EstimationFrame::ConstPtr&)> on_new_frame;

  static CallbackSlot<void(const std::vector<EstimationFrame::ConstPtr>&)> on_update_frames;
  static CallbackSlot<void(const std::vector<EstimationFrame::ConstPtr>&)> on_update_keyframes;
  static CallbackSlot<void(const std::vector<EstimationFrame::ConstPtr>&)> on_marginalized_frames;

  static CallbackSlot<void(gtsam_ext::IncrementalFixedLagSmootherExt&)> on_smoother_update;
};

}  // namespace glim