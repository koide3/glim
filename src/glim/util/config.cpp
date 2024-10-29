#include <glim/util/config.hpp>

#include <boost/filesystem.hpp>
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

GlobalConfig* GlobalConfig::instance(const std::string& config_path, bool override_path) {
  if (inst == nullptr || override_path) {
    if (inst) {
      delete inst;
    }

    inst = new GlobalConfig(config_path + "/config.json");
    inst->override_param("global", "config_path", config_path);
  }
  return inst;
}

std::string GlobalConfig::get_config_path(const std::string& config_name) {
  auto config = instance();
  const std::string directory = config->param<std::string>("global", "config_path", ".");
  const std::string filename = config->param<std::string>("global", config_name, config_name + ".json");
  return directory + "/" + filename;
}

void GlobalConfig::dump(const std::string& path) {
  spdlog::debug("dumping config to {} (config_path={})", path, param<std::string>("global", "config_path", "."));
  boost::filesystem::create_directories(path);
  this->save(path + "/config.json");

  const auto& json = std::any_cast<const nlohmann::json&>(config);
  for (const auto& param : json["global"].items()) {
    const std::string config_name = param.key();
    const std::string config_file = param.value();
    if (config_name == "config_path" || config_name == "config_ext") {
      continue;
    }

    spdlog::debug("dumping {} : {}", config_name, config_file);
    const Config conf(get_config_path(config_name));
    conf.save(path + "/" + config_file);
  }

  spdlog::debug("dumping global config done");
}

}  // namespace glim