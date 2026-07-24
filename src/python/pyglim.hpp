#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include <cmath>
#include <cstdint>
#include <vector>
#include <memory>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace py = pybind11;

using FloatArray = py::array_t<float, py::array::c_style | py::array::forcecast>;
using DoubleArray = py::array_t<double, py::array::c_style | py::array::forcecast>;
using IntArray = py::array_t<int32_t, py::array::c_style | py::array::forcecast>;

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

/// @brief Validate that the leading dimension of a batched array matches the batch size
inline void validate_batch_size(const std::string& name, py::ssize_t dim0, int batch_size) {
  if (dim0 != batch_size) {
    throw std::invalid_argument(name + " batch size mismatch (" + std::to_string(dim0) + " vs " + std::to_string(batch_size) + ")");
  }
}

/// @brief Validate that a batch index is within [0, batch_size)
inline void validate_batch_index(int batch_index, int batch_size) {
  if (batch_index < 0 || batch_index >= batch_size) {
    throw std::out_of_range("batch_index out of range (" + std::to_string(batch_index) + " not in [0, " + std::to_string(batch_size) + "))");
  }
}

/// @brief Convert a per-point scalar array ([N] or [N, 1]) for a single frame into a vector.
inline std::vector<double> convert_scalar_frame(const DoubleArray& values, const std::string& name) {
  if (values.ndim() != 1 && !(values.ndim() == 2 && values.shape(1) == 1)) {
    throw std::invalid_argument(name + " must be [N] or [N, 1] (ndim=" + std::to_string(values.ndim()) + ")");
  }
  return std::vector<double>(values.data(), values.data() + values.shape(0));
}

/// @brief Convert a point array ([N, 3] or [N, 4]) for a single frame into homogeneous points. If
///        keep_last is true and 4 columns are given, the 4th column becomes the homogeneous/alpha
///        component; otherwise it is set to 1.0.
inline std::vector<Eigen::Vector4d> convert_point_frame(const DoubleArray& points, const std::string& name, bool keep_last) {
  if (points.ndim() != 2) {
    throw std::invalid_argument(name + " must be [N, 3] or [N, 4] (ndim=" + std::to_string(points.ndim()) + ")");
  }
  const py::ssize_t cols = points.shape(1);
  if (cols != 3 && cols != 4) {
    throw std::invalid_argument(name + " must have 3 or 4 columns (cols=" + std::to_string(cols) + ")");
  }

  const py::ssize_t num_points = points.shape(0);
  const double* data = points.data();

  std::vector<Eigen::Vector4d> converted(num_points);
  for (py::ssize_t i = 0; i < num_points; i++) {
    const double* p = data + i * cols;
    const double w = (keep_last && cols == 4) ? p[3] : 1.0;
    converted[i] << p[0], p[1], p[2], w;
  }
  return converted;
}

/// @brief Convert a per-point ring array ([N] or [N, 1]) for a single frame into a vector.
inline std::vector<uint32_t> convert_ring_frame(const IntArray& rings) {
  if (rings.ndim() != 1 && !(rings.ndim() == 2 && rings.shape(1) == 1)) {
    throw std::invalid_argument("rings must be [N] or [N, 1] (ndim=" + std::to_string(rings.ndim()) + ")");
  }

  const py::ssize_t num_points = rings.shape(0);
  const int32_t* data = rings.data();

  std::vector<uint32_t> converted(num_points);
  for (py::ssize_t i = 0; i < num_points; i++) {
    converted[i] = static_cast<uint32_t>(data[i]);
  }
  return converted;
}

/// @brief Convert a batched per-point scalar array ([B, N] or [B, N, 1]) into per-batch vectors.
///        Frames with fewer points are padded at the tail with NaN; padding is trimmed.
inline std::vector<std::vector<double>> convert_scalar_batch(const DoubleArray& values, const std::string& name, int batch_size) {
  if (values.ndim() != 2 && !(values.ndim() == 3 && values.shape(2) == 1)) {
    throw std::invalid_argument(name + " must be [B, N] or [B, N, 1] (ndim=" + std::to_string(values.ndim()) + ")");
  }
  validate_batch_size(name, values.shape(0), batch_size);

  const py::ssize_t num_points = values.shape(1);
  const double* data = values.data();

  std::vector<std::vector<double>> converted(batch_size);
  for (int b = 0; b < batch_size; b++) {
    const double* row = data + static_cast<py::ssize_t>(b) * num_points;
    py::ssize_t n = 0;
    while (n < num_points && !std::isnan(row[n])) {
      n++;
    }
    converted[b].assign(row, row + n);
  }
  return converted;
}

/// @brief Convert a batched point array ([B, N, 3] or [B, N, 4]) into per-batch homogeneous points.
///        Frames with fewer points are padded at the tail with NaN; padding is trimmed. If keep_last
///        is true and 4 columns are given, the 4th column becomes the homogeneous/alpha component;
///        otherwise it is set to 1.0.
inline std::vector<std::vector<Eigen::Vector4d>> convert_point_batch(const DoubleArray& points, const std::string& name, int batch_size, bool keep_last) {
  if (points.ndim() != 3) {
    throw std::invalid_argument(name + " must be [B, N, 3] or [B, N, 4] (ndim=" + std::to_string(points.ndim()) + ")");
  }
  const py::ssize_t cols = points.shape(2);
  if (cols != 3 && cols != 4) {
    throw std::invalid_argument(name + " must have 3 or 4 columns (cols=" + std::to_string(cols) + ")");
  }
  validate_batch_size(name, points.shape(0), batch_size);

  const py::ssize_t num_points = points.shape(1);
  const double* data = points.data();

  std::vector<std::vector<Eigen::Vector4d>> converted(batch_size);
  for (int b = 0; b < batch_size; b++) {
    std::vector<Eigen::Vector4d>& out = converted[b];
    out.reserve(num_points);
    const double* row = data + static_cast<py::ssize_t>(b) * num_points * cols;
    for (py::ssize_t i = 0; i < num_points; i++) {
      const double* p = row + i * cols;
      if (std::isnan(p[0])) {
        break;
      }
      const double w = (keep_last && cols == 4) ? p[3] : 1.0;
      out.emplace_back(p[0], p[1], p[2], w);
    }
  }
  return converted;
}

/// @brief Convert a batched per-point ring array ([B, N] or [B, N, 1]) into per-batch vectors.
///        Frames with fewer points are padded at the tail with -1; padding is trimmed.
inline std::vector<std::vector<uint32_t>> convert_ring_batch(const IntArray& rings, int batch_size) {
  if (rings.ndim() != 2 && !(rings.ndim() == 3 && rings.shape(2) == 1)) {
    throw std::invalid_argument("rings must be [B, N] or [B, N, 1] (ndim=" + std::to_string(rings.ndim()) + ")");
  }
  validate_batch_size("rings", rings.shape(0), batch_size);

  const py::ssize_t num_points = rings.shape(1);
  const int32_t* data = rings.data();

  std::vector<std::vector<uint32_t>> converted(batch_size);
  for (int b = 0; b < batch_size; b++) {
    std::vector<uint32_t>& out = converted[b];
    out.reserve(num_points);
    const int32_t* row = data + static_cast<py::ssize_t>(b) * num_points;
    for (py::ssize_t i = 0; i < num_points; i++) {
      if (row[i] < 0) {
        break;
      }
      out.push_back(static_cast<uint32_t>(row[i]));
    }
  }
  return converted;
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
