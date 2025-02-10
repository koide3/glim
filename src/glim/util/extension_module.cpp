#include <glim/util/extension_module.hpp>

#include <spdlog/spdlog.h>
#include <glim/util/load_module.hpp>

namespace glim {

std::shared_ptr<ExtensionModule> ExtensionModule::load_module(const std::string& so_name) {
  return load_module_from_so<ExtensionModule>(so_name, "create_extension_module");
}

void ExtensionModule::export_classes(const std::string& so_name) {
  open_so(so_name);
}

}  // namespace glim