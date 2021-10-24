#include <glim/common/callbacks.hpp>

namespace glim {

CallbackSlot<void(NotificationLevel level, const std::string&)> CommonCallbacks::on_notification;

}