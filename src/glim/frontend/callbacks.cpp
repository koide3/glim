#include <glim/frontend/callbacks.hpp>

namespace glim {

CallbackSlot<void()> OdometryEstimationCallbacks::on_smoother_corruption;
CallbackSlot<void(const EstimationFrame::ConstPtr&)> OdometryEstimationCallbacks::on_new_frame;

CallbackSlot<void(const std::vector<EstimationFrame::ConstPtr>&)> OdometryEstimationCallbacks::on_marginalized_frames;
CallbackSlot<void(const std::vector<EstimationFrame::ConstPtr>&)> OdometryEstimationCallbacks::on_update_frames;
CallbackSlot<void(const std::vector<EstimationFrame::ConstPtr>&)> OdometryEstimationCallbacks::on_update_keyframes;

CallbackSlot<void(gtsam_ext::IncrementalFixedLagSmootherExt&)> OdometryEstimationCallbacks::on_smoother_update;

}  // namespace glim