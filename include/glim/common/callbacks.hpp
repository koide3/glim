#pragma once

#include <string>
#include <glim/util/callback_slot.hpp>

namespace glim {

enum NotificationLevel { INFO, WARNING, ERROR };

/**
 * @brief Callbacks for common processing
 */
struct CommonCallbacks {
  /**
   * @brief Notification callback
   * @param level   Notification level
   * @param message Notification message
   */
  static CallbackSlot<void(NotificationLevel level, const std::string& message)> on_notification;
};

/**
 * @brief Issue a notification message
 * @param level   Notification level
 * @param message Notification message
 */
inline void notify(NotificationLevel level, const std::string& message) {
  CommonCallbacks::on_notification(level, message);
}

}  // namespace glim