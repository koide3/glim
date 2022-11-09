#pragma once

#include <memory>
#include <vector>
#include <iostream>
#include <boost/format.hpp>

#include <Eigen/Core>
#include <gtsam_ext/types/frame.hpp>
#include <glim/util/raw_points.hpp>

#ifdef GLIM_ROS2
#include <sensor_msgs/msg/point_cloud2.hpp>
namespace glim {
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2Ptr = sensor_msgs::msg::PointCloud2::SharedPtr;
using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
using PointField = sensor_msgs::msg::PointField;

template <typename Stamp>
double to_sec(const Stamp& stamp) {
  return stamp.sec + stamp.nanosec / 1e9;
}

inline builtin_interfaces::msg::Time from_sec(const double time) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = std::floor(time);
  stamp.nanosec = (time - stamp.sec) * 1e9;
  return stamp;
}

}  // namespace glim
#else
#include <sensor_msgs/PointCloud2.h>
namespace glim {
using PointCloud2 = sensor_msgs::PointCloud2;
using PointCloud2Ptr = sensor_msgs::PointCloud2::Ptr;
using PointCloud2ConstPtr = sensor_msgs::PointCloud2::ConstPtr;
using PointField = sensor_msgs::PointField;

template <typename Stamp>
double to_sec(const Stamp& stamp) {
  return stamp.toSec();
}

inline ros::Time from_sec(const double time) {
  ros::Time stamp;
  stamp.sec = std::floor(time);
  stamp.nsec = (time - stamp.sec) * 1e9;
  return stamp;
}

}  // namespace glim
#endif

namespace glim {

template <typename T>
Eigen::Vector4d get_vec4(const void* x, const void* y, const void* z) {
  return Eigen::Vector4d(*reinterpret_cast<const T*>(x), *reinterpret_cast<const T*>(y), *reinterpret_cast<const T*>(z), 1.0);
}

static RawPoints::Ptr extract_raw_points(const PointCloud2& points_msg, const std::string& intensity_channel = "intensity") {
  int num_points = points_msg.width * points_msg.height;

  int x_type = 0;
  int y_type = 0;
  int z_type = 0;
  int time_type = 0;  // ouster and livox
  int intensity_type = 0;

  int x_offset = -1;
  int y_offset = -1;
  int z_offset = -1;
  int time_offset = -1;
  int intensity_offset = -1;

  std::unordered_map<std::string, std::pair<int*, int*>> fields;
  fields["x"] = std::make_pair(&x_type, &x_offset);
  fields["y"] = std::make_pair(&y_type, &y_offset);
  fields["z"] = std::make_pair(&z_type, &z_offset);
  fields["t"] = std::make_pair(&time_type, &time_offset);
  fields["time"] = std::make_pair(&time_type, &time_offset);
  fields["time_stamp"] = std::make_pair(&time_type, &time_offset);
  fields["timestamp"] = std::make_pair(&time_type, &time_offset);
  fields[intensity_channel] = std::make_pair(&intensity_type, &intensity_offset);

  for (const auto& field : points_msg.fields) {
    auto found = fields.find(field.name);
    if (found == fields.end()) {
      continue;
    }

    *found->second.first = field.datatype;
    *found->second.second = field.offset;
  }

  if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
    std::cerr << "waning: missing point coordinate fields" << std::endl;
    return nullptr;
  }

  if ((x_type != PointField::FLOAT32 && x_type != PointField::FLOAT64) || x_type != y_type || x_type != y_type) {
    std::cerr << "warning: unsupported points type" << std::endl;
    return nullptr;
  }

  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> points;
  points.resize(num_points);
  for (int i = 0; i < num_points; i++) {
    const auto* x_ptr = &points_msg.data[points_msg.point_step * i + x_offset];
    const auto* y_ptr = &points_msg.data[points_msg.point_step * i + y_offset];
    const auto* z_ptr = &points_msg.data[points_msg.point_step * i + z_offset];

    if (x_type == PointField::FLOAT32) {
      points[i] = get_vec4<float>(x_ptr, y_ptr, z_ptr);
    } else {
      points[i] = get_vec4<double>(x_ptr, y_ptr, z_ptr);
    }
  }

  std::vector<double> times;
  if (time_offset >= 0) {
    times.resize(num_points);

    for (int i = 0; i < num_points; i++) {
      const auto* time_ptr = &points_msg.data[points_msg.point_step * i + time_offset];
      switch (time_type) {
        case PointField::UINT32:
          times[i] = *reinterpret_cast<const uint32_t*>(time_ptr) / 1e9;
          break;
        case PointField::FLOAT32:
          times[i] = *reinterpret_cast<const float*>(time_ptr);
          break;
        case PointField::FLOAT64:
          times[i] = *reinterpret_cast<const double*>(time_ptr);
          break;
        default:
          std::cerr << "warning: unsupported time type " << time_type << std::endl;
          return nullptr;
      }
    }
  }

  std::vector<double> intensities;
  if (intensity_offset >= 0) {
    intensities.resize(num_points);

    for (int i = 0; i < num_points; i++) {
      const auto* intensity_ptr = &points_msg.data[points_msg.point_step * i + intensity_offset];
      switch (intensity_type) {
        case PointField::UINT8:
          intensities[i] = *reinterpret_cast<const std::uint8_t*>(intensity_ptr);
          break;
        case PointField::UINT16:
          intensities[i] = *reinterpret_cast<const std::uint16_t*>(intensity_ptr);
          break;
        case PointField::UINT32:
          intensities[i] = *reinterpret_cast<const std::uint32_t*>(intensity_ptr);
          break;
        case PointField::FLOAT32:
          intensities[i] = *reinterpret_cast<const float*>(intensity_ptr);
          break;
        case PointField::FLOAT64:
          intensities[i] = *reinterpret_cast<const double*>(intensity_ptr);
          break;
        default:
          std::cerr << "warning: unsupported intensity type" << std::endl;
          return nullptr;
      }
    }
  }

  const double stamp = to_sec(points_msg.header.stamp);
  return RawPoints::Ptr(new RawPoints{stamp, times, intensities, points});
}

static RawPoints::Ptr extract_raw_points(const PointCloud2ConstPtr& points_msg, const std::string& intensity_channel = "intensity") {
  return extract_raw_points(*points_msg);
}

static PointCloud2ConstPtr frame_to_pointcloud2(const std::string& frame_id, const double stamp, const gtsam_ext::Frame& frame) {
  PointCloud2Ptr msg(new PointCloud2);
  msg->header.frame_id = frame_id;
  msg->header.stamp = from_sec(stamp);

  msg->width = frame.size();
  msg->height = 1;

  std::vector<std::string> field_names = {"x", "y", "z", "t"};
  int num_fields = frame.times ? 4 : 3;
  msg->fields.resize(num_fields);

  for (int i = 0; i < num_fields; i++) {
    msg->fields[i].name = field_names[i];
    msg->fields[i].offset = sizeof(float) * i;
    msg->fields[i].datatype = PointField::FLOAT32;
    msg->fields[i].count = 1;
  }

  msg->is_bigendian = false;
  msg->point_step = sizeof(float) * num_fields;
  msg->row_step = sizeof(float) * num_fields * frame.size();

  msg->data.resize(sizeof(float) * num_fields * frame.size());
  for (int i = 0; i < frame.size(); i++) {
    float* point = reinterpret_cast<float*>(msg->data.data() + msg->point_step * i);
    for (int j = 0; j < 3; j++) {
      point[j] = frame.points[i][j];
    }

    if (frame.times) {
      point[3] = frame.times[i];
    }
  }

  return msg;
}

}  // namespace glim