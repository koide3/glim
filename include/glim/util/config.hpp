#pragma once

#include <any>
#include <memory>
#include <optional>

namespace glim {

class Config {
public:
  Config(const std::string& config_filename);
  virtual ~Config();

  template <typename T>
  std::optional<T> param(const std::string& module_name, const std::string& param_name) const;

  template <typename T>
  T param(const std::string& module_name, const std::string& param_name, const T& default_value) const;

  template <typename T>
  bool override_param(const std::string& module_name, const std::string& param_name, const T& value);

private:
  std::any config;
};

class GlobalConfig : public Config {
private:
  GlobalConfig() : Config("") {}
  virtual ~GlobalConfig() override {}

public:
  static GlobalConfig* instance() {
    if (inst == nullptr) {
      inst = new GlobalConfig();
    }
    return inst;
  }

  static std::string get_config_path(const std::string& config_name) {
    auto config = instance();
    const std::string directory = config->param<std::string>("global", "config_path", ".");
    const std::string filename = config->param<std::string>("global", config_name, config_name + ".json");
    return directory + "/" + filename;
  }

  static GlobalConfig* inst;
};

}  // namespace glim