#include <glim/preprocess/callbacks.hpp>

namespace glim {
CallbackSlot<void(const RawPoints::ConstPtr& points)> PreprocessCallbacks::on_raw_points_received;

CallbackSlot<void(gtsam_points::PointCloudCPU::Ptr& points)> PreprocessCallbacks::on_preprocessing_begin;
CallbackSlot<void(gtsam_points::PointCloudCPU::Ptr& points)> PreprocessCallbacks::on_downsampling_finished;
CallbackSlot<void(gtsam_points::PointCloudCPU::Ptr& points)> PreprocessCallbacks::on_filtering_finished;
}  // namespace glim