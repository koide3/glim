#pragma once

#include <atomic>
#include <thread>
#include <glim/backend/global_mapping.hpp>
#include <glim/util/concurrent_vector.hpp>

namespace glim {

/**
 * @brief Global mapping executor to wrap and asynchronously run a global mapping object
 * @note  All the exposed public methods except for save() are thread-safe
 *
 */
class AsyncGlobalMapping {
public:
  /**
   * @brief Construct a new Async Global Mapping object
   * @param global_mapping  Global mapping object
   */
  AsyncGlobalMapping(const std::shared_ptr<glim::GlobalMappingBase>& global_mapping);

  /**
   * @brief Destroy the Async Global Mapping object
   */
  ~AsyncGlobalMapping();

  /**
   * @brief Insert an image
   * @param stamp   Timestamp
   * @param image   Image
   */
  void insert_image(const double stamp, const cv::Mat& image);

  /**
   * @brief Insert an IMU frame
   * @param stamp         Timestamp
   * @param linear_acc    Linear acceleration
   * @param angular_vel   Angular velocity
   */
  void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);

  /**
   * @brief Insert a SubMap
   * @param submap  SubMap
   */
  void insert_submap(const SubMap::Ptr& submap);

  /**
   * @brief Wait for the global mapping thread
   */
  void join();

  /**
   * @brief Number of data in the input queue (for load control)
   * @return Input queue size
   */
  int input_queue_size() const;

  /**
   * @brief Number of data in the output queue
   * @return Output queue size
   */
  int output_queue_size() const;

  /**
   * @brief Save the mapping result
   * @note  This method may not be thread-safe and is expected to be called after join()
   * @param path    Save path
   */
  void save(const std::string& path);

private:
  void run();

private:
  std::atomic_bool kill_switch;      // Flag to stop the thread immediately (Hard kill switch)
  std::atomic_bool end_of_sequence;  // Flag to stop the thread when the input queues become empty (Soft kill switch)
  std::thread thread;

  ConcurrentVector<std::pair<double, cv::Mat>> input_image_queue;
  ConcurrentVector<Eigen::Matrix<double, 7, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 7, 1>>> input_imu_queue;
  ConcurrentVector<SubMap::Ptr> input_submap_queue;

  std::atomic_bool request_to_optimize;

  std::shared_ptr<glim::GlobalMappingBase> global_mapping;
};

}  // namespace  glim
