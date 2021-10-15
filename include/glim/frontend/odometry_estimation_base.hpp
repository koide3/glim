#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <glim/frontend/estimation_frame.hpp>
#include <glim/preprocess/preprocessed_frame.hpp>

namespace glim {

class OdometryEstimationBase {
public:
  virtual ~OdometryEstimationBase() {}

  virtual void insert_imu(double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {}
  virtual void insert_image(const cv::Mat& image) {}
  virtual EstimationFrame::ConstPtr insert_frame(PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::Ptr>& marginalized_states) = 0;
};

}  // namespace glim