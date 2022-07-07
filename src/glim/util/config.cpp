#include <glim/util/config.hpp>

#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <nlohmann/json.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/util/console_colors.hpp>

namespace glim {

GlobalConfig* GlobalConfig::inst = nullptr;

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

template<>
struct traits<std::vector<Eigen::Isometry3d>> {
  using InType = std::vector<double>;
  using OutType = std::vector<Eigen::Isometry3d>;

  static std::optional<OutType> convert(const InType& in) {
    if (in.size() % 7) {
      return std::nullopt;
    }

    OutType poses(in.size() / 7);
    for (int i = 0; i < poses.size(); i++) {
      poses[i].setIdentity();
      poses[i].translation() << in[i * 7], in[i * 7 + 1], in[i * 7 + 2];
      poses[i].linear() = Eigen::Quaterniond(in[i * 7 + 6], in[i * 7 + 3], in[i * 7 + 4], in[i * 7 + 5]).normalized().toRotationMatrix();
    }

    return poses;
  }

  static std::vector<double> invert(const OutType& value) {
    std::vector<double> values(value.size() * 7);
    for (int i = 0; i < value.size(); i++) {
      Eigen::Vector3d t = value[i].translation();
      Eigen::Quaterniond q(value[i].linear());

      values[i * 7] = t.x();
      values[i * 7 + 1] = t.y();
      values[i * 7 + 2] = t.z();
      values[i * 7 + 3] = q.x();
      values[i * 7 + 4] = q.y();
      values[i * 7 + 5] = q.z();
      values[i * 7 + 6] = q.w();
    }

    return values;
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
T Config::param_cast(const std::string& module_name, const std::string& param_name) const {
  auto found = param<T>(module_name, param_name);
  if (!found) {
    std::cerr << console::bold_red;
    std::cerr << "error : param " << console::underline << module_name << "/" << param_name << console::reset << console::bold_red << " not found" << std::endl;
    std::cerr << console::reset;
    abort();
  }
  return *found;
}

template <typename T>
bool Config::override_param(const std::string& module_name, const std::string& param_name, const T& value) {
  auto& json = std::any_cast<nlohmann::json&>(config);

  json[module_name][param_name] = traits<T>::invert(value);

  return true;
}

template <typename T>
std::optional<T> Config::param_nested(const std::vector<std::string>& nested_module_names, const std::string& param_name) const {
  const auto& json = std::any_cast<const nlohmann::json&>(config);

  nlohmann::json::const_iterator itr = json.find(nested_module_names[0]);
  if (itr == json.end()) {
    return std::nullopt;
  }

  for (int i = 1; i < nested_module_names.size(); i++) {
    const auto next = itr->find(nested_module_names[i]);
    if (next == itr->end()) {
      return std::nullopt;
    }

    itr = next;
  }

  auto parameter = itr->find(param_name);
  if (parameter == itr->end()) {
    return std::nullopt;
  }

  return traits<T>::convert(parameter->get<typename traits<T>::InType>());
}

template <typename T>
T Config::param_nested(const std::vector<std::string>& nested_module_names, const std::string& param_name, const T& default_value) const {
  auto found = param_nested<T>(nested_module_names, param_name);
  if (!found) {
    std::cerr << console::yellow;
    std::cerr << "warning: param " << console::underline;
    for (const auto& module_name : nested_module_names) {
      std::cerr << module_name << "/";
    }
    std::cerr << param_name << console::reset << console::yellow << " not found" << std::endl;
    std::cerr << "       : use the default value" << std::endl;
    std::cerr << console::reset;
    return default_value;
  }

  return found.value();
}

template <typename T>
T Config::param_cast_nested(const std::vector<std::string>& nested_module_names, const std::string& param_name) const {
  auto found = param_nested<T>(nested_module_names, param_name);
  if (!found) {
    std::cerr << console::bold_red;
    std::cerr << "error: param " << console::underline;
    for (const auto& module_name : nested_module_names) {
      std::cerr << module_name << "/";
    }
    std::cerr << param_name << console::reset << console::bold_red << " not found" << std::endl;
    std::cerr << console::reset;
    abort();
  }
  return *found;
}

void Config::save(const std::string& path) const {
  const auto& json = std::any_cast<const nlohmann::json&>(config);

  std::ofstream ofs(path);
  ofs << std::setw(2) << json << std::endl;
}

#define DEFINE_SPECIALIZATION(TYPE)                                                                           \
  template TYPE Config::param(const std::string&, const std::string&, const TYPE&) const;                     \
  template TYPE Config::param_cast(const std::string&, const std::string&) const;                             \
  template TYPE Config::param_nested(const std::vector<std::string>&, const std::string&, const TYPE&) const; \
  template TYPE Config::param_cast_nested(const std::vector<std::string>&, const std::string&) const;         \
  template bool Config::override_param(const std::string&, const std::string&, const TYPE&);

DEFINE_SPECIALIZATION(bool)
DEFINE_SPECIALIZATION(int)
DEFINE_SPECIALIZATION(float)
DEFINE_SPECIALIZATION(double)
DEFINE_SPECIALIZATION(std::string)
DEFINE_SPECIALIZATION(std::vector<int>)
DEFINE_SPECIALIZATION(std::vector<double>)
DEFINE_SPECIALIZATION(std::vector<std::string>)

DEFINE_SPECIALIZATION(Eigen::Vector2d)
DEFINE_SPECIALIZATION(Eigen::Vector3d)
DEFINE_SPECIALIZATION(Eigen::Vector4d)
DEFINE_SPECIALIZATION(Eigen::Quaterniond)
DEFINE_SPECIALIZATION(Eigen::Isometry3d)

DEFINE_SPECIALIZATION(std::vector<Eigen::Isometry3d>)

}  // namespace glim