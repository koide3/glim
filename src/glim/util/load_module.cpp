#include <glim/util/load_module.hpp>

#include <dlfcn.h>
#include <spdlog/spdlog.h>

namespace glim {

void open_so(const std::string& so_name) {
  void* handle = dlopen(so_name.c_str(), RTLD_LAZY);
  if (handle == nullptr) {
    spdlog::warn("failed to open {}", so_name);
    spdlog::warn("{}", dlerror());
  }
}

void* load_symbol(const std::string& so_name, const std::string& symbol_name) {
  void* handle = dlopen(so_name.c_str(), RTLD_LAZY);
  if (handle == nullptr) {
    spdlog::warn("failed to open {}", so_name);
    spdlog::warn("{}", dlerror());
    return nullptr;
  }

  auto* func = dlsym(handle, symbol_name.c_str());
  if (func == nullptr) {
    spdlog::warn("failed to find symbol={} in {}", symbol_name, so_name);
    spdlog::warn("{}", dlerror());
  }

  return func;
}

}  // namespace glim
