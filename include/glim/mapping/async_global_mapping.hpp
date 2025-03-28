#pragma once

#include <atomic>
#include <mutex>
#include <thread>
#include <glim/mapping/global_mapping.hpp>
#include <glim/util/concurrent_vector.hpp>

namespace spdlog {
class logger;
}

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
   * @param global_mapping         Global mapping object
   * @param optimization_interval  Optimizer is updated every this interval even if no additional values and factors are given
   */
  AsyncGlobalMapping(const std::shared_ptr<glim::GlobalMappingBase>& global_mapping, const int optimization_interval_sec = 5);

  /**
   * @brief Destroy the Async Global Mapping object
   */
  ~AsyncGlobalMapping();

#ifdef GLIM_USE_OPENCV
  /**
   * @brief Insert an image
   * @param stamp   Timestamp
   * @param image   Image
   */
  void insert_image(const double stamp, const cv::Mat& image);
#endif

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
  int workload() const;

  /**
   * @brief Save the mapping result
   * @note  This method may not be thread-safe and is expected to be called after join()
   * @param path    Save path
   */
  void save(const std::string& path);

  std::vector<Eigen::Vector4d> export_points();

  std::shared_ptr<glim::GlobalMappingBase> get_global_mapping() {
    std::lock_guard<std::mutex> lock(global_mapping_mutex);
    return global_mapping;
  }

private:
  void run();

private:
  std::atomic_bool kill_switch;      ///< Flag to stop the thread immediately (Hard kill switch)
  std::atomic_bool end_of_sequence;  ///< Flag to stop the thread when the input queues become empty (Soft kill switch)
  std::thread thread;

#ifdef GLIM_USE_OPENCV
  ConcurrentVector<std::pair<double, cv::Mat>> input_image_queue;
#endif
  ConcurrentVector<Eigen::Matrix<double, 7, 1>> input_imu_queue;
  ConcurrentVector<SubMap::Ptr> input_submap_queue;

  int optimization_interval;
  std::atomic_bool request_to_optimize;
  std::atomic_bool request_to_recover;
  std::atomic<double> request_to_find_overlapping_submaps;

  std::mutex global_mapping_mutex;
  std::shared_ptr<glim::GlobalMappingBase> global_mapping;

  // Logging
  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace  glim
