#pragma once

#include <spdlog/spdlog.h>

namespace glim {

void set_default_logger(const std::shared_ptr<spdlog::logger>& logger);

}  // namespace glim
