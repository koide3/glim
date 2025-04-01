#include <glim/preprocess/callbacks.hpp>

namespace glim {
CallbackSlot<void(const RawPoints::ConstPtr& points)> PreprocessCallbacks::on_raw_points_received;
}  // namespace glim