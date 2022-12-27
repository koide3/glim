#include <glim/util/logging.hpp>
#include <spdlog/spdlog.h>

namespace glim {

void set_default_logger(const std::shared_ptr<spdlog::logger>& logger) {
  spdlog::set_default_logger(logger);
}

}  // namespace glim
