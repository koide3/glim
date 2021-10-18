#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include <glim/frontend/estimation_frame.hpp>
#include <glim/backend/sub_map.hpp>

namespace glim {

class SubMappingBase {
public:
  virtual ~SubMappingBase() {}

  virtual void insert_image(const double stamp, const cv::Mat& image);
  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);
  virtual void insert_frame(const EstimationFrame::ConstPtr& frame);

  virtual std::vector<SubMap::Ptr> get_submaps() = 0;
};

}