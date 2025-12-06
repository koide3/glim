#include <filesystem>
#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>

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

  const Config config(glim::GlobalConfig::get_config_path("config_logging"));
  const std::string log_dir = config.param<std::string>("logging", "log_dir", std::string("/tmp"));
  const std::string log_filename = module_name == "glim" ? "main" : module_name;

  if (!std::filesystem::exists(log_dir)) {
    std::filesystem::create_directories(log_dir);
  }

  logger = spdlog::stdout_color_mt(module_name);
  logger->sinks().push_back(get_ringbuffer_sink());

  if (!config.param<bool>("logging", "save_logs", true)) {
    return logger;
  }

  if (config.param<bool>("logging", "rotate_logs", true)) {
    const size_t max_file_size_kb = config.param<int>("logging", "max_file_size_kb", 8192);
    const size_t max_file_size_bytes = max_file_size_kb * 1024;
    const size_t max_files = config.param<int>("logging", "max_files", 10);

    auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(log_dir + "/glim_" + log_filename + ".log", max_file_size_bytes, max_files);
    logger->sinks().push_back(rotating_sink);
  } else {
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_dir + "/glim_" + log_filename + ".log", true);
    logger->sinks().push_back(file_sink);
  }

  logger->set_level(get_default_logger()->level());

  return logger;
}

}  // namespace glim
