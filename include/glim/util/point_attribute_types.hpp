#pragma once

#include <map>
#include <limits>
#include <string>
#include <vector>
#include <cstdint>
#include <cstring>
#include <optional>
#include <stdexcept>

namespace glim {

/**
 * @brief Scalar type of an extra point attribute (values are identical to sensor_msgs/PointField datatypes)
 */
enum class PointFieldType : std::uint8_t { INT8 = 1, UINT8 = 2, INT16 = 3, UINT16 = 4, INT32 = 5, UINT32 = 6, FLOAT32 = 7, FLOAT64 = 8 };

/**
 * @brief Call func with a value-initialized instance of the C++ type corresponding to the point field type
 *        e.g., visit_point_field_type(type, [](auto tag) { using T = decltype(tag); ... });
 */
template <typename Func>
decltype(auto) visit_point_field_type(const PointFieldType type, Func&& func) {
  switch (type) {
    case PointFieldType::INT8:
      return func(std::int8_t());
    case PointFieldType::UINT8:
      return func(std::uint8_t());
    case PointFieldType::INT16:
      return func(std::int16_t());
    case PointFieldType::UINT16:
      return func(std::uint16_t());
    case PointFieldType::INT32:
      return func(std::int32_t());
    case PointFieldType::UINT32:
      return func(std::uint32_t());
    case PointFieldType::FLOAT32:
      return func(float());
    case PointFieldType::FLOAT64:
      return func(double());
  }
  throw std::invalid_argument("invalid point field type " + std::to_string(static_cast<int>(type)));
}

/// @brief Check if a raw datatype value (e.g., PointField::datatype) is a valid point field type
inline bool is_valid_point_field_type(const int datatype) {
  return datatype >= static_cast<int>(PointFieldType::INT8) && datatype <= static_cast<int>(PointFieldType::FLOAT64);
}

/// @brief Size of a point field type in bytes
inline size_t point_field_type_size(const PointFieldType type) {
  return visit_point_field_type(type, [](auto tag) { return sizeof(tag); });
}

/// @brief Point field type name ("int8", "uint8", ..., "float32", "float64")
inline std::string point_field_type_name(const PointFieldType type) {
  static const char* names[] = {"int8", "uint8", "int16", "uint16", "int32", "uint32", "float32", "float64"};
  return names[static_cast<int>(type) - 1];
}

/// @brief Parse a point field type name
inline std::optional<PointFieldType> point_field_type_from_string(const std::string& name) {
  for (int i = static_cast<int>(PointFieldType::INT8); i <= static_cast<int>(PointFieldType::FLOAT64); i++) {
    if (point_field_type_name(static_cast<PointFieldType>(i)) == name) {
      return static_cast<PointFieldType>(i);
    }
  }
  return std::nullopt;
}

/**
 * @brief Extra per-point attribute values stored in their native type
 */
struct PointAttribute {
public:
  PointAttribute() : type(PointFieldType::FLOAT32) {}
  PointAttribute(const PointFieldType type, const size_t num_points) : type(type), data(num_points * point_field_type_size(type)) {}

  /// Number of values
  size_t size() const { return data.size() / point_field_type_size(type); }
  /// Size of each value in bytes
  size_t elem_size() const { return point_field_type_size(type); }

  template <typename T>
  const T* values() const {
    return reinterpret_cast<const T*>(data.data());
  }

  template <typename T>
  T* values() {
    return reinterpret_cast<T*>(data.data());
  }

  /// @brief Get the i-th value converted to double
  double get_as_double(const size_t i) const {
    return visit_point_field_type(type, [&](auto tag) { return static_cast<double>(values<decltype(tag)>()[i]); });
  }

  /// @brief Convert the values to another type (values are cast via double)
  PointAttribute convert_to(const PointFieldType new_type) const {
    PointAttribute converted(new_type, size());
    visit_point_field_type(new_type, [&](auto tag) {
      using T = decltype(tag);
      for (size_t i = 0; i < size(); i++) {
        converted.values<T>()[i] = static_cast<T>(get_as_double(i));
      }
    });
    return converted;
  }

  /// @brief Create an attribute filled with a value that indicates "no data" (NaN for floating point types, 0 for integer types)
  static PointAttribute invalid(const PointFieldType type, const size_t num_points) {
    PointAttribute attribute(type, num_points);
    visit_point_field_type(type, [&](auto tag) {
      using T = decltype(tag);
      const T value = std::numeric_limits<T>::has_quiet_NaN ? std::numeric_limits<T>::quiet_NaN() : T(0);
      std::fill(attribute.values<T>(), attribute.values<T>() + num_points, value);
    });
    return attribute;
  }

public:
  PointFieldType type;             ///< Value type
  std::vector<std::uint8_t> data;  ///< Raw values (elem_size() bytes per point)
};

/// @brief Extra per-point attributes (e.g., "red", "green", "blue") keyed by attribute name
using PointAttributes = std::map<std::string, PointAttribute>;

/// @brief Value types of extra per-point attributes keyed by attribute name
using PointAttributeTypes = std::map<std::string, PointFieldType>;

}  // namespace glim
