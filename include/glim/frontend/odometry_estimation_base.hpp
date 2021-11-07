#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <glim/frontend/estimation_frame.hpp>
#include <glim/preprocess/preprocessed_frame.hpp>

namespace glim {

/**
 * @brief Odometry estimation base class
 *
 */
class OdometryEstimationBase {
public:
  virtual ~OdometryEstimationBase() {}

  /**
   * @brief Insert an image
   * @param stamp   Timestamp
   * @param image   Image
   */
  virtual void insert_image(const double stamp, const cv::Mat& image);

  /**
   * @brief Insert an IMU data
   * @param stamp        Timestamp
   * @param linear_acc   Linear acceleration
   * @param angular_vel  Angular velocity
   */
  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);

  /**
   * @brief Insert a point cloud
   * @param frame                       Preprocessed point cloud
   * @param marginalized_states         [out] Marginalized estimation frames
   * @return EstimationFrame::ConstPtr  Estimation result for the latest frame
   */
  virtual EstimationFrame::ConstPtr insert_frame(const PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::ConstPtr>& marginalized_states);

  /**
   * @brief Pop out the remaining non-marginalized frames (called at the end of the sequence)
   * @return std::vector<EstimationFrame::ConstPtr>
   */
  virtual std::vector<EstimationFrame::ConstPtr> get_remaining_frames() { return std::vector<EstimationFrame::ConstPtr>(); }
};

}  // namespace glim