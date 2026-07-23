#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include <vector>
#include <memory>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace py = pybind11;

using DoubleArray = py::array_t<double, py::array::c_style | py::array::forcecast>;

/// @brief Validate and convert a numpy array [N, 3] or [N, 4] into homogeneous points
inline std::vector<Eigen::Vector4d> convert_points(const DoubleArray& points) {
  if (points.ndim() != 2) {
    throw std::invalid_argument("points must be 2-dimensional (ndim=" + std::to_string(points.ndim()) + ")");
  }
  if (points.shape(1) != 3 && points.shape(1) != 4) {
    throw std::invalid_argument("points must have 3 or 4 columns (cols=" + std::to_string(points.shape(1)) + ")");
  }

  const int num_points = points.shape(0);
  const int cols = points.shape(1);
  const double* data = points.data();

  std::vector<Eigen::Vector4d> converted(num_points);
  for (int i = 0; i < num_points; i++) {
    converted[i] << data[i * cols], data[i * cols + 1], data[i * cols + 2], 1.0;
  }
  return converted;
}

/// @brief Convert homogeneous points into a numpy array [N, 3]
inline py::array_t<double> convert_points(const Eigen::Vector4d* points, const size_t num_points) {
  py::array_t<double> arr({static_cast<py::ssize_t>(num_points), static_cast<py::ssize_t>(3)});
  auto view = arr.mutable_unchecked<2>();
  for (size_t i = 0; i < num_points; i++) {
    view(i, 0) = points[i].x();
    view(i, 1) = points[i].y();
    view(i, 2) = points[i].z();
  }
  return arr;
}

inline py::array_t<double> convert_points(const std::vector<Eigen::Vector4d>& points) {
  return convert_points(points.data(), points.size());
}

/// @brief Validate a numpy array [N] holding per-point scalar attributes (e.g., times and intensities)
inline std::vector<double> convert_scalars(const DoubleArray& values, const std::string& name, const int expected_size) {
  if (values.ndim() != 1) {
    throw std::invalid_argument(name + " must be 1-dimensional (ndim=" + std::to_string(values.ndim()) + ")");
  }
  if (values.shape(0) != expected_size) {
    throw std::invalid_argument(name + " must have the same number of elements as points (" + std::to_string(values.shape(0)) + " vs " + std::to_string(expected_size) + ")");
  }
  return std::vector<double>(values.data(), values.data() + values.shape(0));
}

/// @brief Validate an IMU data batch [N, 7] (t, ax, ay, az, wx, wy, wz)
inline const DoubleArray& validate_imu_batch(const DoubleArray& imu_data) {
  if (imu_data.ndim() != 2) {
    throw std::invalid_argument("imu_data must be 2-dimensional (ndim=" + std::to_string(imu_data.ndim()) + ")");
  }
  if (imu_data.shape(1) != 7) {
    throw std::invalid_argument("imu_data must have 7 columns [t, ax, ay, az, wx, wy, wz] (cols=" + std::to_string(imu_data.shape(1)) + ")");
  }
  return imu_data;
}

/// @brief Feed an IMU data batch [N, 7] to a module with insert_imu(stamp, linear_acc, angular_vel)
template <typename Module>
void insert_imu_batch(Module& module, const DoubleArray& imu_data) {
  validate_imu_batch(imu_data);
  const auto view = imu_data.template unchecked<2>();

  py::gil_scoped_release release;
  for (py::ssize_t i = 0; i < view.shape(0); i++) {
    const double stamp = view(i, 0);
    const Eigen::Vector3d linear_acc(view(i, 1), view(i, 2), view(i, 3));
    const Eigen::Vector3d angular_vel(view(i, 4), view(i, 5), view(i, 6));
    module.insert_imu(stamp, linear_acc, angular_vel);
  }
}

/// @brief Cast away constness of a shared_ptr for passing const objects to Python.
///        Bound classes expose read-only accessors, so this is safe in practice.
template <typename T>
std::shared_ptr<T> unconst(const std::shared_ptr<const T>& ptr) {
  return std::const_pointer_cast<T>(ptr);
}

template <typename T>
std::vector<std::shared_ptr<T>> unconst(const std::vector<std::shared_ptr<const T>>& ptrs) {
  std::vector<std::shared_ptr<T>> converted(ptrs.size());
  std::transform(ptrs.begin(), ptrs.end(), converted.begin(), [](const auto& ptr) { return std::const_pointer_cast<T>(ptr); });
  return converted;
}

/// @brief Define a property to access an Eigen::Isometry3d member as a 4x4 matrix
template <typename Class, typename... Extra>
auto& def_isometry_property(py::class_<Class, Extra...>& cls, const char* name, Eigen::Isometry3d Class::* member, const char* doc = "") {
  return cls.def_property(
    name,
    [member](const Class& self) -> Eigen::Matrix4d { return (self.*member).matrix(); },
    [member](Class& self, const Eigen::Matrix4d& mat) { (self.*member) = Eigen::Isometry3d(mat); },
    doc);
}
