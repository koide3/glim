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
   * @brief Check if the module is behind the main mapping process.
   */
  virtual bool needs_wait() const { return false; }

  /**
   * @brief Check if the module is alive. (If it returns false, the system will be shutdown)
   */
  virtual bool ok() const { return true; }

  /**
   * @brief Load an extension module from a dynamic library
   * @param so_name  Dynamic library name
   * @return         Loaded extension module
   */
  static std::shared_ptr<ExtensionModule> load_module(const std::string& so_name);
};

}  // namespace glim