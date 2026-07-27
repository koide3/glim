#include <glim/common/callbacks.hpp>

namespace glim {

CallbackSlot<void(const std::string& module_name, const GenericDataContainer::ConstPtr& data)> CommonCallbacks::on_extension_output;

}  // namespace glim