#pragma once

#include <glim/util/config.hpp>

#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/util/convert_to_string.hpp>

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

template <>
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
    spdlog::warn("param {}/{} not found", module_name, param_name);
    spdlog::warn("use default_value={}", convert_to_string(default_value));
    return default_value;
  }

  spdlog::debug("param {}/{}={}", module_name, param_name, convert_to_string(found.value()));
  return found.value();
}

template <typename T>
T Config::param_cast(const std::string& module_name, const std::string& param_name) const {
  auto found = param<T>(module_name, param_name);
  if (!found) {
    spdlog::critical("param {}/{} not found", module_name, param_name);
    abort();
  }

  spdlog::debug("param {}/{}={}", module_name, param_name, convert_to_string(found.value()));
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
    std::stringstream param_name;
    for (const auto& module_name : nested_module_names) {
      param_name << module_name << "/";
    }
    spdlog::warn("param {} not found", param_name.str());
    spdlog::warn("use default_value={}", convert_to_string(default_value));
    return default_value;
  }

  return found.value();
}

template <typename T>
T Config::param_cast_nested(const std::vector<std::string>& nested_module_names, const std::string& param_name) const {
  auto found = param_nested<T>(nested_module_names, param_name);
  if (!found) {
    std::stringstream param_name;
    for (const auto& module_name : nested_module_names) {
      param_name << module_name << "/";
    }
    spdlog::critical("param {} not found", param_name.str());
    abort();
  }
  return *found;
}

#define DEFINE_CONFIG_IO_SPECIALIZATION(TYPE)                                                                                                  \
  template std::optional<TYPE> Config::param(const std::string& module_name, const std::string& param_name) const;                             \
  template TYPE Config::param(const std::string&, const std::string&, const TYPE&) const;                                                      \
  template TYPE Config::param_cast(const std::string&, const std::string&) const;                                                              \
  template std::optional<TYPE> Config::param_nested(const std::vector<std::string>& nested_module_names, const std::string& param_name) const; \
  template TYPE Config::param_nested(const std::vector<std::string>&, const std::string&, const TYPE&) const;                                  \
  template TYPE Config::param_cast_nested(const std::vector<std::string>&, const std::string&) const;                                          \
  template bool Config::override_param(const std::string&, const std::string&, const TYPE&);

}  // namespace glim
