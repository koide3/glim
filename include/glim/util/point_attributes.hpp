#pragma once

#include <string>
#include <vector>
#include <cstring>
#include <algorithm>
#include <stdexcept>
#include <spdlog/spdlog.h>
#include <gtsam_points/types/point_cloud.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim/util/config.hpp>
#include <glim/util/point_attribute_types.hpp>

namespace glim {

/**
 * @brief Load the names of extra point fields to be passed through the pipeline (sensors/extra_point_fields).
 * @note  Names must not contain '_' because gtsam_points::PointCloudCPU::load() cannot restore such aux attributes.
 * @param config_sensors  Sensor config
 * @return                Names of extra point fields
 */
inline std::vector<std::string> load_extra_point_fields(const Config& config_sensors) {
  const auto fields = config_sensors.param<std::vector<std::string>>("sensors", "extra_point_fields", {});
  for (const auto& field : fields) {
    if (field.empty() || field.find('_') != std::string::npos || field.find(':') != std::string::npos) {
      throw std::runtime_error("invalid extra_point_fields entry '" + field + "' (must be non-empty and must not contain '_' or ':')");
    }
  }
  return fields;
}

/**
 * @brief Get the value types of extra point attributes recorded in the global config (meta/point_attribute_types).
 *        Types are detected from the input data and recorded by the preprocessor, and they are dumped with the map.
 */
inline PointAttributeTypes get_point_attribute_types() {
  PointAttributeTypes types;
  const auto entries = GlobalConfig::instance()->param<std::vector<std::string>>("meta", "point_attribute_types", {});
  for (const auto& entry : entries) {
    const size_t pos = entry.find(':');
    const auto type = pos == std::string::npos ? std::nullopt : point_field_type_from_string(entry.substr(pos + 1));
    if (!type) {
      spdlog::warn("invalid point attribute type entry '{}'", entry);
      continue;
    }
    types[entry.substr(0, pos)] = *type;
  }
  return types;
}

/**
 * @brief Record the value types of extra point attributes in the global config (meta/point_attribute_types)
 */
inline void set_point_attribute_types(const PointAttributeTypes& types) {
  std::vector<std::string> entries;
  for (const auto& [name, type] : types) {
    entries.emplace_back(name + ":" + point_field_type_name(type));
  }
  GlobalConfig::instance()->override_param<std::vector<std::string>>("meta", "point_attribute_types", entries);
}

/**
 * @brief Attach point attributes to a point cloud as aux attributes (raw values in their native types)
 * @param frame       Point cloud
 * @param attributes  Point attributes (each must have frame.size() elements)
 */
inline void add_point_attributes(gtsam_points::PointCloudCPU& frame, const PointAttributes& attributes) {
  for (const auto& [name, attribute] : attributes) {
    if (attribute.size() != frame.size()) {
      spdlog::warn("point attribute size mismatch (name={} attribute={} points={})", name, attribute.size(), frame.size());
      continue;
    }

    auto storage = std::make_shared<std::vector<std::uint8_t>>(attribute.data);
    frame.aux_attributes_storage[name] = storage;
    frame.aux_attributes[name] = std::make_pair(attribute.elem_size(), storage->data());
  }
}

/**
 * @brief Extract point attributes from aux attributes of a point cloud
 * @param frame  Point cloud
 * @param types  Names and value types of attributes to be extracted (missing attributes and attributes with mismatching sizes are skipped)
 * @return       Point attributes
 */
inline PointAttributes get_point_attributes(const gtsam_points::PointCloud& frame, const PointAttributeTypes& types) {
  PointAttributes attributes;
  for (const auto& [name, type] : types) {
    const auto found = frame.aux_attributes.find(name);
    if (found == frame.aux_attributes.end() || found->second.first != point_field_type_size(type)) {
      continue;
    }

    PointAttribute attribute(type, frame.size());
    std::memcpy(attribute.data.data(), found->second.second, attribute.data.size());
    attributes[name] = std::move(attribute);
  }
  return attributes;
}

/**
 * @brief Concatenate aux attributes of point clouds and attach them to the merged point cloud.
 *        Only attributes that exist in all frames with the same element size are concatenated.
 * @param frames  Point clouds to be concatenated (in the same order as the points of merged)
 * @param merged  Concatenated point cloud
 */
template <typename PointCloudPtr>
void concat_aux_attributes(const std::vector<PointCloudPtr>& frames, gtsam_points::PointCloudCPU& merged) {
  if (frames.empty()) {
    return;
  }

  for (const auto& [name, attrib] : frames.front()->aux_attributes) {
    const size_t elem_size = attrib.first;
    const bool available = std::all_of(frames.begin(), frames.end(), [&](const auto& frame) {
      const auto found = frame->aux_attributes.find(name);
      return found != frame->aux_attributes.end() && found->second.first == elem_size;
    });
    if (!available) {
      continue;
    }

    auto storage = std::make_shared<std::vector<unsigned char>>(elem_size * merged.size());
    size_t offset = 0;
    for (const auto& frame : frames) {
      const size_t bytes = elem_size * frame->size();
      if (offset + bytes > storage->size()) {
        spdlog::warn("aux attribute size mismatch while concatenating (name={})", name);
        break;
      }
      std::memcpy(storage->data() + offset, frame->aux_attributes.at(name).second, bytes);
      offset += bytes;
    }

    if (offset != storage->size()) {
      continue;
    }

    merged.aux_attributes_storage[name] = storage;
    merged.aux_attributes[name] = std::make_pair(elem_size, storage->data());
  }
}

}  // namespace glim
