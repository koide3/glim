#pragma once

#include <mutex>
#include <thread>
#include <atomic>

#include <glim/util/concurrent_vector.hpp>
#include <glim/frontend/odometry_estimation_base.hpp>

namespace glim {

/**
 * @brief Odometry estimation executor to wrap and asynchronously run OdometryEstimationBase
 * @note  All the exposed public methods are thread-safe
 *
 */
class AsyncOdometryEstimation {
public:
  /**
   * @brief Construct a new Async Odometry Estimation object
   * @param odometry_estimation  Odometry estimation to be wrapped
   */
  AsyncOdometryEstimation(const std::shared_ptr<OdometryEstimationBase>& odometry_estimation, bool enable_imu);

  /**
   * @brief Destroy the Async Odometry Estimation object
   */
  ~AsyncOdometryEstimation();

  /**
   * @brief Insert an image into the odometry estimation
   * @param stamp   Timestamp
   * @param image   Image
   */
  void insert_image(const double stamp, const cv::Mat& image);

  /**
   * @brief Insert an IMU data into the odometry estimation
   * @param stamp         Timestamp
   * @param linear_acc    Linear acceleration
   * @param angular_vel   Angular velocity
   */
  void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);

  /**
   * @brief Insert a preprocessed point cloud into odometry estimation
   * @param frame  Preprocessed point cloud
   */
  void insert_frame(const PreprocessedFrame::Ptr& frame);

  /**
   * @brief Wait for the odometry estimation thread
   */
  void join();

  /**
   * @brief  Size of the input data queue size (for load control)
   * @return int Input queue size
   */
  int input_queue_size() const;

  /**
   * @brief  Size of the output data queue size
   * @return int Output queue size
   */
  int output_queue_size() const;

  /**
   * @brief Get the estimation results
   * @param estimation_results    Estimation results
   * @param marginalized_frames   Marginalized frames
   */
  void get_results(std::vector<EstimationFrame::ConstPtr>& estimation_results, std::vector<EstimationFrame::ConstPtr>& marginalized_frames);

private:
  void run();

private:
  std::atomic_bool kill_switch;      // Flag to stop the thread immediately (Hard kill switch)
  std::atomic_bool end_of_sequence;  // Flag to stop the thread when the input queues become empty (Soft kill switch)
  std::thread thread;

  // Input queues
  ConcurrentVector<std::pair<double, cv::Mat>> input_image_queue;
  ConcurrentVector<Eigen::Matrix<double, 7, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 7, 1>>> input_imu_queue;
  ConcurrentVector<PreprocessedFrame::Ptr> input_frame_queue;

  // Output queues
  ConcurrentVector<EstimationFrame::ConstPtr> output_estimation_results;
  ConcurrentVector<EstimationFrame::ConstPtr> output_marginalized_frames;

  bool enable_imu;
  std::shared_ptr<OdometryEstimationBase> odometry_estimation;
};

}  // namespace glim