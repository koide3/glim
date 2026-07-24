#include <glim/util/callback_slot.hpp>

namespace glim {

thread_local int CallbackContext::current_id = CallbackContext::GLOBAL;

}