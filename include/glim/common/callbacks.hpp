#pragma once

#include <glim/util/callback_slot.hpp>
#include <glim/util/generic_data_container.hpp>

namespace glim {

/**
 * @brief Callbacks commonly used across the framework
 *
 */
struct CommonCallbacks {
  /// @brief Callback for GLIM extension output
  /// @param data  GenericDataContainer containing output data of extension modules
  static CallbackSlot<void(const std::string& module_name, const GenericDataContainer::ConstPtr& data)> on_extension_output;
};

}  // namespace glim