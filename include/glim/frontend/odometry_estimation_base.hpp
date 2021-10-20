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

  virtual void insert_image(const double stamp, const cv::Mat& image);
  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);
  virtual EstimationFrame::ConstPtr insert_frame(const PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::ConstPtr>& marginalized_states);

  virtual std::vector<EstimationFrame::ConstPtr> get_remaining_frames() { return std::vector<EstimationFrame::ConstPtr>(); }
};

}  // namespace glim