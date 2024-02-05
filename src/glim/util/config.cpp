#include <glim/util/config.hpp>

#include <glim/util/config_impl.hpp>

namespace glim {

GlobalConfig* GlobalConfig::inst = nullptr;

Config::Config(const std::string& config_filename) {
  nlohmann::json json;
  if (config_filename.empty()) {
    config = json;
    return;
  }

  std::ifstream ifs(config_filename);
  if (!ifs) {
    spdlog::error("failed to open {}", config_filename);
  } else {
    json = nlohmann::json::parse(ifs, nullptr, true, true);
  }

  config = json;
}

Config::~Config() {}

void Config::save(const std::string& path) const {
  const auto& json = std::any_cast<const nlohmann::json&>(config);

  std::ofstream ofs(path);
  ofs << std::setw(2) << json << std::endl;
}

DEFINE_CONFIG_IO_SPECIALIZATION(bool)
DEFINE_CONFIG_IO_SPECIALIZATION(int)
DEFINE_CONFIG_IO_SPECIALIZATION(float)
DEFINE_CONFIG_IO_SPECIALIZATION(double)
DEFINE_CONFIG_IO_SPECIALIZATION(std::string)
DEFINE_CONFIG_IO_SPECIALIZATION(std::vector<bool>)
DEFINE_CONFIG_IO_SPECIALIZATION(std::vector<int>)
DEFINE_CONFIG_IO_SPECIALIZATION(std::vector<double>)
DEFINE_CONFIG_IO_SPECIALIZATION(std::vector<std::string>)

DEFINE_CONFIG_IO_SPECIALIZATION(Eigen::Vector2d)
DEFINE_CONFIG_IO_SPECIALIZATION(Eigen::Vector3d)
DEFINE_CONFIG_IO_SPECIALIZATION(Eigen::Vector4d)
DEFINE_CONFIG_IO_SPECIALIZATION(Eigen::Quaterniond)
DEFINE_CONFIG_IO_SPECIALIZATION(Eigen::Isometry3d)

DEFINE_CONFIG_IO_SPECIALIZATION(std::vector<Eigen::Isometry3d>)

}  // namespace glim