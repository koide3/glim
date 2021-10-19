#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include <glim/backend/sub_map.hpp>

namespace glim {

class GlobalMappingBase {
public:
  virtual ~GlobalMappingBase() {}

  virtual void insert_image(const double stamp, const cv::Mat& image);
  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);
  virtual void insert_submap(const SubMap::Ptr& submap);
};
}