#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/sinks/ringbuffer_sink.h>

namespace glim {

std::shared_ptr<spdlog::logger> get_default_logger();

void set_default_logger(const std::shared_ptr<spdlog::logger>& logger);

std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt> get_ringbuffer_sink(int buffer_size = 128);

std::shared_ptr<spdlog::logger> create_module_logger(const std::string& module_name);

}  // namespace glim
