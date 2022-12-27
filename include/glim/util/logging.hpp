#pragma once

#include <spdlog/spdlog.h>

namespace glim {

std::shared_ptr<spdlog::logger> get_default_logger();

void set_default_logger(const std::shared_ptr<spdlog::logger>& logger);

}  // namespace glim
