#include <unistd.h>
#include <thread>
#include <fstream>
#include <iostream>
#include <glim/util/logging.hpp>
#include <glim/util/extension_module.hpp>
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

#ifdef BUILD_GTSAM_POINTS_GPU
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

      double vm_usage, resident_set;
      mem_usage(vm_usage, resident_set);
      logger->info("Memory usage: {:.2f} / {:.2f} MB", vm_usage / 1024.0, resident_set / 1024.0);
    }
  }

  void mem_usage(double& vm_usage, double& resident_set) {
    vm_usage = 0.0;
    resident_set = 0.0;
    std::ifstream stat_stream("/proc/self/stat", std::ios_base::in);  // get info from proc directory
    // create some variables to get info
    std::string pid, comm, state, ppid, pgrp, session, tty_nr;
    std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
    std::string utime, stime, cutime, cstime, priority, nice;
    std::string O, itrealvalue, starttime;
    unsigned long vsize;
    long rss;
    stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt >> utime >> stime >> cutime >> cstime >>
      priority >> nice >> O >> itrealvalue >> starttime >> vsize >> rss;
    stat_stream.close();
    long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024;
    vm_usage = vsize / 1024.0;
    resident_set = rss * page_size_kb;
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