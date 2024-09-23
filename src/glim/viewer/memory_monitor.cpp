#include <unistd.h>
#include <thread>
#include <fstream>
#include <sstream>
#include <iostream>
#include <glim/util/logging.hpp>
#include <glim/util/extension_module.hpp>
#include <gtsam_points/config.hpp>
#include <gtsam_points/cuda/cuda_memory.hpp>

namespace glim {

class MemoryMonitor : public ExtensionModule {
public:
  MemoryMonitor() : logger(create_module_logger("mem")) {
    kill_switch = false;
    thread = std::thread([this] { task(); });
  }

  ~MemoryMonitor() {
    kill_switch = true;
    thread.join();
  }

  void task() {
    auto last_update_time = std::chrono::high_resolution_clock::now();
    while (!kill_switch) {
      std::this_thread::sleep_for(std::chrono::seconds(1));

      auto now = std::chrono::high_resolution_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - last_update_time).count() < 5) {
        continue;
      }
      last_update_time = now;

#ifdef GTSAM_POINTS_USE_CUDA
      size_t gpu_free = 0;
      size_t gpu_total = 0;
      gtsam_points::cuda_mem_get_info(&gpu_free, &gpu_total);
      const double gpu_used = static_cast<double>(gpu_total - gpu_free) / gpu_total;

      if (gpu_used > 0.8) {
        const size_t gpu_used_mb = (gpu_total - gpu_free) / 1024 / 1024;
        const size_t gpu_total_mb = gpu_total / 1024 / 1024;
        logger->warn("GPU memory usage: {}/{} MB {:.2f}%", gpu_used_mb, gpu_total_mb, gpu_used * 100);
      }
#endif

      size_t mem_free_kb, mem_total_kb;
      mem_usage(mem_free_kb, mem_total_kb);

      const double cpu_used = static_cast<double>(mem_total_kb - mem_free_kb) / mem_total_kb;

      if (cpu_used > 0.8) {
        const double used_mb = static_cast<double>(mem_total_kb - mem_free_kb) / 1024.0;
        const double total_mb = static_cast<double>(mem_total_kb) / 1024.0;
        logger->warn("CPU memory usage: {:.2f} / {:.2f} MB {:.2f}%", used_mb, total_mb, cpu_used * 100);
      }

      size_t rss_kb, shared_kb;
      rss_mem_usage(rss_kb, shared_kb);

      const double rss_mb = static_cast<double>(rss_kb) / 1024.0;
      const double shared_mb = static_cast<double>(shared_kb) / 1024.0;
      logger->debug("RSS: {:.2f} MB, Shared: {:.2f} MB", rss_mb, shared_mb);
    }
  }

  void mem_usage(size_t& mem_free_kb, size_t& mem_total_kb) {
    mem_free_kb = 0;
    mem_total_kb = 0;

    std::ifstream ifs("/proc/meminfo");
    if (!ifs) {
      logger->warn("Failed to open /proc/meminfo");
      return;
    }

    std::string line;
    while (!ifs.eof() && std::getline(ifs, line) && !line.empty()) {
      std::istringstream sst(line);
      std::string name;
      if (line.find("MemTotal:") != std::string::npos) {
        sst >> name >> mem_total_kb;
      } else if (line.find("MemAvailable:") != std::string::npos) {
        sst >> name >> mem_free_kb;
      }

      if (mem_free_kb > 0 && mem_total_kb > 0) {
        break;
      }
    }
  }

  void rss_mem_usage(size_t& rss_kb, size_t& shared_kb) {
    rss_kb = 0;
    shared_kb = 0;

    std::ifstream ifs("/proc/self/statm");
    if (!ifs) {
      logger->warn("Failed to open /proc/self/statm");
      return;
    }

    size_t size, resident, share;
    ifs >> size >> resident >> share;

    long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024;
    rss_kb = resident * page_size_kb;
    shared_kb = share * page_size_kb;
  }

private:
  std::atomic_bool kill_switch;
  std::thread thread;

  // Logging
  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::MemoryMonitor();
}