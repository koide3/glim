#include <glim/util/logging.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace glim {

std::shared_ptr<spdlog::logger> get_default_logger() {
  return spdlog::default_logger();
}

void set_default_logger(const std::shared_ptr<spdlog::logger>& logger) {
  spdlog::set_default_logger(logger);
}

namespace {
std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt> ringbuffer_sink;
}

std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt> get_ringbuffer_sink(int buffer_size) {
  if (!ringbuffer_sink) {
    ringbuffer_sink = std::make_shared<spdlog::sinks::ringbuffer_sink_mt>(buffer_size);
  }
  return ringbuffer_sink;
}

std::shared_ptr<spdlog::logger> create_module_logger(const std::string& module_name) {
  std::shared_ptr<spdlog::logger> logger = spdlog::get(module_name);
  if (logger) {
    return logger;
  }

  logger = spdlog::stdout_color_mt(module_name);
  logger->sinks().push_back(get_ringbuffer_sink());

  if (get_default_logger()->level() < spdlog::level::info) {
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("/tmp/glim_" + module_name + ".log", true);
    logger->sinks().push_back(file_sink);
    logger->set_level(get_default_logger()->level());
  }

  return logger;
}

}  // namespace glim
