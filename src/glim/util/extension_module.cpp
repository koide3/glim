#include <glim/util/extension_module.hpp>

#include <dlfcn.h>
#include <spdlog/spdlog.h>
#include <glim/util/console_colors.hpp>

namespace glim {

std::shared_ptr<ExtensionModule> ExtensionModule::load(const std::string& so_name) {
  void* handle = dlopen(so_name.c_str(), RTLD_LAZY);
  if (handle == nullptr) {
    spdlog::warn("failed to open {}", so_name);
    spdlog::warn("{}", dlerror());
    return nullptr;
  }

  auto create_module = (ExtensionModule * (*)()) dlsym(handle, "create_extension_module");
  auto ext_module = create_module();

  return std::shared_ptr<ExtensionModule>(ext_module);
}

}  // namespace glim