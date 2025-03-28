#include <glim/viewer/callbacks.hpp>

namespace glim {
CallbackSlot<void(const std::function<void(guik::LightViewer& viewer)>& callback)> ViewerCallbacks::request_to_invoke;
}