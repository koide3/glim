#pragma once

#include <string>
#include <vector>
#include <fmt/format.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glim {

// Convertion to string

template <typename T>
std::string convert_to_string(const T& value) {
  return fmt::format("{}", value);
}

template <typename T2>
std::string convert_to_string(const std::vector<T2>& values) {
  std::stringstream sst;
  sst << "[";
  for (int i = 0; i < values.size(); i++) {
    if (i) {
      sst << ",";
    }
    sst << convert_to_string(values[i]);
  }
  sst << "]";
  return sst.str();
}

template <int D>
std::string convert_to_string(const Eigen::Matrix<double, D, 1>& value) {
  std::stringstream sst;
  sst << "vec(";
  for (int i = 0; i < value.size(); i++) {
    if (i) {
      sst << ",";
    }
    sst << fmt::format("{:.6f}", value[i]);
  }
  sst << ")";
  return sst.str();
}

template <>
inline std::string convert_to_string(const Eigen::Quaterniond& quat) {
  return fmt::format("quat({:.6f},{:.6f},{:.6f},{:.6f})", quat.x(), quat.y(), quat.z(), quat.w());
}

template <>
inline std::string convert_to_string(const Eigen::Isometry3d& pose) {
  const Eigen::Vector3d trans(pose.translation());
  const Eigen::Quaterniond quat(pose.linear());
  return fmt::format("se3({:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f})", trans.x(), trans.y(), trans.z(), quat.x(), quat.y(), quat.z(), quat.w());
}
}  // namespace glim