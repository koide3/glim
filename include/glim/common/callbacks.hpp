#pragma once

#include <glim/util/callback_slot.hpp>

namespace glim {

enum NotificationLevel { INFO, WARNING, ERROR };

struct CommonCallbacks {
  static CallbackSlot<void(NotificationLevel level, const std::string&)> on_notification;
};

inline void notify(NotificationLevel level, const std::string& message) {
  CommonCallbacks::on_notification(level, message);
}

}  // namespace glim