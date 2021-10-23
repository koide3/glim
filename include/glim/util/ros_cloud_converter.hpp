#pragma once

#include <memory>
#include <vector>
#include <iostream>
#include <boost/format.hpp>

#include <Eigen/Core>
#include <gtsam_ext/types/frame.hpp>

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

builtin_interfaces::msg::Time from_sec(const double time) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = std::floor(time);
  stamp.nanosec = (time - stamp.sec) * 1e9;
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

ros::Time from_sec(const double time) {
  ros::Time stamp;
  stamp.secs = std::floor(time);
  stamp.nsecs = (time - stamp.secs) * 1e9;
}

}  // namespace glim
#endif

namespace glim {

template <typename T>
Eigen::Vector4d get_vec4(const void* x, const void* y, const void* z) {
  return Eigen::Vector4d(*reinterpret_cast<const T*>(x), *reinterpret_cast<const T*>(y), *reinterpret_cast<const T*>(z), 1.0);
}

struct RawPoints {
public:
  using Ptr = std::shared_ptr<RawPoints>;
  using ConstPtr = std::shared_ptr<const RawPoints>;

  static RawPoints::Ptr extract(const PointCloud2ConstPtr& points_msg) {
    int num_points = points_msg->width * points_msg->height;

    int x_type = 0;
    int y_type = 0;
    int z_type = 0;
    int time_type = 0;        // ouster and livox
    int time_stamp_type = 0;  // for hesai lidar
    int intensity_type = 0;

    int x_offset = -1;
    int y_offset = -1;
    int z_offset = -1;
    int time_offset = -1;
    int time_stamp_offset = -1;
    int intensity_offset = -1;

    std::unordered_map<std::string, std::pair<int*, int*>> fields;
    fields["x"] = std::make_pair(&x_type, &x_offset);
    fields["y"] = std::make_pair(&y_type, &y_offset);
    fields["z"] = std::make_pair(&z_type, &z_offset);
    fields["time"] = std::make_pair(&time_type, &time_offset);
    fields["time_stamp"] = std::make_pair(&time_stamp_type, &time_stamp_offset);
    fields["intensity"] = std::make_pair(&intensity_type, &intensity_offset);

    for (const auto& field : points_msg->fields) {
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

    const auto FLOAT32 = PointField::FLOAT32;
    const auto FLOAT64 = PointField::FLOAT64;
    if ((x_type != FLOAT32 && x_type != FLOAT64) || x_type != y_type || x_type != y_type) {
      std::cerr << "warning: unsupported points type" << std::endl;
      return nullptr;
    }

    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> points;
    points.resize(num_points);
    for (int i = 0; i < num_points; i++) {
      const auto* x_ptr = &points_msg->data[points_msg->point_step * i + x_offset];
      const auto* y_ptr = &points_msg->data[points_msg->point_step * i + y_offset];
      const auto* z_ptr = &points_msg->data[points_msg->point_step * i + z_offset];

      if (x_type == FLOAT32) {
        points[i] = get_vec4<float>(x_ptr, y_ptr, z_ptr);
      } else {
        points[i] = get_vec4<double>(x_ptr, y_ptr, z_ptr);
      }
    }

    std::vector<double> times;
    if (time_offset >= 0) {
      times.resize(num_points);

      for (int i = 0; i < num_points; i++) {
        const auto* time_ptr = &points_msg->data[points_msg->point_step * i + time_offset];
        if (time_type == FLOAT32) {
          times[i] = *reinterpret_cast<const float*>(time_ptr);
        } else if (time_type == FLOAT64) {
          times[i] = *reinterpret_cast<const double*>(time_ptr);
        } else {
          std::cerr << "warning: unsupported time type" << std::endl;
          return nullptr;
        }
      }
    }

    if (time_stamp_offset >= 0) {
      times.resize(num_points);
      for (int i = 0; i < num_points; i++) {
        const auto* time_ptr = &points_msg->data[points_msg->point_step * i + time_stamp_offset];
        if (time_stamp_type == FLOAT32) {
          times[i] = *reinterpret_cast<const float*>(time_ptr);
        } else if (time_stamp_type == FLOAT64) {
          times[i] = *reinterpret_cast<const double*>(time_ptr);
        } else {
          std::cerr << "warning: unsupported time type" << std::endl;
          return nullptr;
        }
      }

      if (times.front() > 1.0) {
        static bool is_first = true;
        if (is_first) {
          is_first = false;
          std::cerr << "warning: a large point timestamp (> 1s) is found " << boost::format("%.6f") % times.front() << std::endl;
          std::cerr << "warning: assume they are abs times and convert them to rel times" << std::endl;
        }

        const double t0 = times.front();

        for (auto& t : times) {
          t = t - t0;
        }
      }
    }

    std::vector<double> intensities;
    if (intensity_offset >= 0) {
      intensities.resize(num_points);

      for (int i = 0; i < num_points; i++) {
        const auto* intensity_ptr = &points_msg->data[points_msg->point_step * i + intensity_offset];
        if (intensity_type == FLOAT32) {
          intensities[i] = *reinterpret_cast<const float*>(intensity_ptr);
        } else if (intensity_type == FLOAT64) {
          intensities[i] = *reinterpret_cast<const double*>(intensity_ptr);
        } else {
          std::cerr << "warning: unsupported intensity type" << std::endl;
          return nullptr;
        }
      }
    }

    const double stamp = to_sec(points_msg->header.stamp);
    return RawPoints::Ptr(new RawPoints{stamp, times, intensities, points});
  }

public:
  double stamp;
  std::vector<double> times;
  std::vector<double> intensities;
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> points;
};

PointCloud2ConstPtr frame_to_pointcloud2(const std::string& frame_id, const double stamp, const gtsam_ext::Frame& frame) {
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