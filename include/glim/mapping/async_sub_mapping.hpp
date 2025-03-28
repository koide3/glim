#pragma once

#include <atomic>
#include <thread>
#include <glim/mapping/sub_mapping.hpp>
#include <glim/util/concurrent_vector.hpp>

namespace glim {

/**
 * @brief SubMapping executor to wrap and asynchronously run a sub mapping object
 * @note  All the exposed public methods are thread-safe
 */
class AsyncSubMapping {
public:
  /**
   * @brief Construct a new Async Sub Mapping object
   * @param sub_mapping sub mapping object
   */
  AsyncSubMapping(const std::shared_ptr<glim::SubMappingBase>& sub_mapping);

  /**
   * @brief Destroy the Async Sub Mapping object
   *
   */
  ~AsyncSubMapping();

#ifdef GLIM_USE_OPENCV
  /**
   * @brief Insert an image
   * @param stamp   Timestamp
   * @param image   Image
   */
  void insert_image(const double stamp, const cv::Mat& image);
#endif

  /**
   * @brief Insert an IMU data
   * @param stamp         Timestamp
   * @param linear_acc    Linear acceleration
   * @param angular_vel   Angular velocity
   */
  void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);

  /**
   * @brief Insert an odometry estimation frame
   * @param odom_frame  Marginalized odometry estimation frame
   */
  void insert_frame(const EstimationFrame::ConstPtr& odom_frame);

  /**
   * @brief Wait for the sub mapping thread
   *
   */
  void join();

  /**
   * @brief Number of data in the input queue (for load control)
   * @return Input queue size
   */
  int workload() const;

  /**
   * @brief Get the created submaps
   * @return Created submaps
   */
  std::vector<SubMap::Ptr> get_results();

private:
  void run();

private:
  std::atomic_bool kill_switch;      // Flag to stop the thread immediately (Hard kill switch)
  std::atomic_bool end_of_sequence;  // Flag to stop the thread when the input queues become empty (Soft kill switch)
  std::thread thread;

#ifdef GLIM_USE_OPENCV
  ConcurrentVector<std::pair<double, cv::Mat>> input_image_queue;
#endif
  ConcurrentVector<Eigen::Matrix<double, 7, 1>> input_imu_queue;
  ConcurrentVector<EstimationFrame::ConstPtr> input_frame_queue;

  ConcurrentVector<SubMap::Ptr> output_submap_queue;

  std::shared_ptr<glim::SubMappingBase> sub_mapping;
};

}  // namespace  glim
