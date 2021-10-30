#include <glim/util/config.hpp>

#include <fstream>
#include <sstream>
#include <iostream>
#include <nlohmann/json.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/util/console_colors.hpp>

namespace glim {

namespace {
// IO traits
template <typename T>
struct traits {
  using InType = T;
  using OutType = T;

  static std::optional<OutType> convert(const InType& in) { return in; }

  static InType invert(const OutType& value) { return value; }
};

// General Eigen IO
template <typename T, int N, int M>
struct traits<Eigen::Matrix<T, N, M>> {
  using InType = std::vector<double>;
  using OutType = Eigen::Matrix<T, N, M>;

  static std::optional<OutType> convert(const InType& in) {
    if (in.size() != N * M) {
      return std::nullopt;
    }
    return Eigen::Map<const OutType>(in.data());
  }

  static std::vector<double> invert(const OutType& value) { return std::vector<double>(value.data(), value.data() + N * M); }
};

// Eigen Quaternion IO
template <typename T>
struct traits<Eigen::Quaternion<T>> {
  using InType = std::vector<double>;
  using OutType = Eigen::Quaternion<T>;

  static std::optional<OutType> convert(const InType& in) {
    if (in.size() != 4) {
      return std::nullopt;
    }

    return OutType(in.data()).normalized();
  }

  static std::vector<double> invert(const OutType& value) { return std::vector<double>{value.x(), value.y(), value.z(), value.w()}; }
};

// Eigen Isometry
template <typename T>
struct traits<Eigen::Transform<T, 3, Eigen::Isometry>> {
  using InType = std::vector<double>;
  using OutType = Eigen::Transform<T, 3, Eigen::Isometry>;

  static std::optional<OutType> convert(const InType& in) {
    if (in.size() != 7) {
      return std::nullopt;
    }

    OutType se3 = OutType::Identity();
    se3.translation() = Eigen::Map<const Eigen::Matrix<T, 3, 1>>(in.data());
    se3.linear() = Eigen::Quaternion<T>(in.data() + 3).normalized().toRotationMatrix();
    return se3;
  }

  static std::vector<double> invert(const OutType& value) {
    Eigen::Matrix<T, 3, 1> t = value.translation();
    Eigen::Quaternion<T> q(value.linear());
    return std::vector<double>{t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w()};
  }
};

}  // namespace

Config::Config(const std::string& config_filename) {
  nlohmann::json json;
  if (config_filename.empty()) {
    config = json;
    return;
  }

  std::ifstream ifs(config_filename);
  if (!ifs) {
    std::cerr << console::bold_red << "error: failed to open " << config_filename << console::reset << std::endl;
  } else {
    json = nlohmann::json::parse(ifs, nullptr, true, true);
  }

  config = json;
}

Config::~Config() {}

template <typename T>
std::optional<T> Config::param(const std::string& module_name, const std::string& param_name) const {
  const auto& json = std::any_cast<const nlohmann::json&>(config);

  auto module = json.find(module_name);
  if (module == json.end()) {
    return std::nullopt;
  }

  auto parameter = module->find(param_name);
  if (parameter == module->end()) {
    return std::nullopt;
  }

  return traits<T>::convert(parameter->get<typename traits<T>::InType>());
}

template <typename T>
T Config::param(const std::string& module_name, const std::string& param_name, const T& default_value) const {
  auto found = param<T>(module_name, param_name);
  if (!found) {
    std::cerr << console::yellow;
    std::cerr << "warning: param " << console::underline << module_name << "/" << param_name << console::reset << console::yellow << " not found" << std::endl;
    std::cerr << "       : use the default value" << std::endl;
    std::cerr << console::reset;
    return default_value;
  }

  return found.value();
}

template <typename T>
bool Config::override_param(const std::string& module_name, const std::string& param_name, const T& value) {
  auto& json = std::any_cast<nlohmann::json&>(config);

  json[module_name][param_name] = traits<T>::invert(value);

  return true;
}

template bool Config::param(const std::string&, const std::string&, const bool&) const;
template int Config::param(const std::string&, const std::string&, const int&) const;
template float Config::param(const std::string&, const std::string&, const float&) const;
template double Config::param(const std::string&, const std::string&, const double&) const;
template std::string Config::param(const std::string&, const std::string&, const std::string&) const;
template std::vector<int> Config::param(const std::string&, const std::string&, const std::vector<int>&) const;
template std::vector<double> Config::param(const std::string&, const std::string&, const std::vector<double>&) const;

template Eigen::Vector2d Config::param(const std::string&, const std::string&, const Eigen::Vector2d&) const;
template Eigen::Vector3d Config::param(const std::string&, const std::string&, const Eigen::Vector3d&) const;
template Eigen::Vector4d Config::param(const std::string&, const std::string&, const Eigen::Vector4d&) const;
template Eigen::Quaterniond Config::param(const std::string&, const std::string&, const Eigen::Quaterniond&) const;
template Eigen::Isometry3d Config::param(const std::string&, const std::string&, const Eigen::Isometry3d&) const;

template bool Config::override_param(const std::string&, const std::string&, const bool&);
template bool Config::override_param(const std::string&, const std::string&, const int&);
template bool Config::override_param(const std::string&, const std::string&, const float&);
template bool Config::override_param(const std::string&, const std::string&, const double&);
template bool Config::override_param(const std::string&, const std::string&, const std::string&);
template bool Config::override_param(const std::string&, const std::string&, const std::vector<int>&);
template bool Config::override_param(const std::string&, const std::string&, const std::vector<double>&);

template bool Config::override_param(const std::string&, const std::string&, const Eigen::Vector2d&);
template bool Config::override_param(const std::string&, const std::string&, const Eigen::Vector3d&);
template bool Config::override_param(const std::string&, const std::string&, const Eigen::Vector4d&);
template bool Config::override_param(const std::string&, const std::string&, const Eigen::Quaterniond&);
template bool Config::override_param(const std::string&, const std::string&, const Eigen::Isometry3d&);

GlobalConfig* GlobalConfig::inst = nullptr;

}  // namespace glim