#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#ifdef GLIM_USE_OPENCV
#include <opencv2/core.hpp>
#endif

#include <glim/odometry/estimation_frame.hpp>
#include <glim/mapping/sub_map.hpp>

namespace spdlog {
class logger;
}

namespace glim {

/**
 * @brief Sub mapping base class
 *
 */
class SubMappingBase {
public:
  SubMappingBase();
  virtual ~SubMappingBase() {}

#ifdef GLIM_USE_OPENCV
  /**
   * @brief Insert an image
   * @param stamp   Timestamp
   * @param image   Image
   */
  virtual void insert_image(const double stamp, const cv::Mat& image);
#endif

  /**
   * @brief Insert an IMU data
   * @param stamp         Timestamp
   * @param linear_acc    Linear acceleration
   * @param angular_vel   Angular velocity
   */
  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);

  /**
   * @brief Insert an odometry estimation frame
   * @param odom_frame  Marginalized odometry estimation frame
   */
  virtual void insert_frame(const EstimationFrame::ConstPtr& frame);

  /**
   * @brief Get the created submaps
   * @return Created submaps
   */
  virtual std::vector<SubMap::Ptr> get_submaps() = 0;

  /**
   * @brief Submit the signal to tell end of sequence and collect the remaining submap data
   * @return std::vector<SubMap::Ptr>
   */
  virtual std::vector<SubMap::Ptr> submit_end_of_sequence() { return std::vector<SubMap::Ptr>(); }

  /**
   * @brief Load a sub mapping module from a shared library
   * @param so_name  Shared library name
   * @return         Loaded sub mapping module
   */
  static std::shared_ptr<SubMappingBase> load_module(const std::string& so_name);

protected:
  // Logging
  std::shared_ptr<spdlog::logger> logger;
};
}  // namespace glim