#pragma once

#include <glim/util/callback_slot.hpp>

namespace guik {
class LightViewer;
}

namespace glim {
struct ViewerCallbacks {
  /**
   * @brief Invoke callback on viewer thread
   * @param callback  Callback to be invoked
   */
  static CallbackSlot<void(const std::function<void(guik::LightViewer& viewer)>& callback)> request_to_invoke;
};
}  // namespace glim