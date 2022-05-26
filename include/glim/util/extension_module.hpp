#pragma once

#include <memory>

namespace glim {

/**
 * @brief Extension module to be dynamically loaded via dynamic linking
 */
class ExtensionModule {
public:
  ExtensionModule() {}
  virtual ~ExtensionModule() {}

  /**
   * @brief Load an extension module from a dynamic library
   * @param so_name  Dynamic library name
   * @return         Loaded extension module
   */
  static std::shared_ptr<ExtensionModule> load(const std::string& so_name);
};

}  // namespace glim