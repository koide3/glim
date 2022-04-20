#pragma once

#include <memory>

namespace glim {

class ExtensionModule {
public:
  ExtensionModule() {}
  virtual ~ExtensionModule() {}

  static std::shared_ptr<ExtensionModule> load(const std::string& so_name);
};

}  // namespace glim