#pragma once

#include <memory>

namespace glim {

void open_so(const std::string& so_name);

void* load_symbol(const std::string& so_name, const std::string& symbol_name);

template <typename Module>
std::shared_ptr<Module> load_module_from_so(const std::string& so_name, const std::string& func_name) {
  auto func = (Module * (*)()) load_symbol(so_name, func_name);
  if (func == nullptr) {
    return nullptr;
  }

  return std::shared_ptr<Module>(func());
}

}  // namespace glim
