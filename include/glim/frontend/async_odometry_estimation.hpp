#pragma once

#include <mutex>
#include <thread>
#include <atomic>

#include <glim/util/concurrent_vector.hpp>
#include <glim/frontend/odometry_estimation_base.hpp>

namespace glim {

class AsyncOdometryEstimation {
public:
  AsyncOdometryEstimation(const std::shared_ptr<OdometryEstimationBase>& odometry_estimation);
  ~AsyncOdometryEstimation();

  void insert_image(const double stamp, const cv::Mat& image);
  void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);
  void insert_frame(const PreprocessedFrame::Ptr& frame);

  void join();

  int input_queue_size() const;
  int output_queue_size() const;

  void get_results(std::vector<EstimationFrame::ConstPtr>& estimation_results, std::vector<EstimationFrame::ConstPtr>& marginalized_frames);

private:
  void run();

private:
  std::atomic_bool kill_switch;      // Flag to stop the thread immediately (Hard kill switch)
  std::atomic_bool end_of_sequence;  // Flag to stop the thread when the input queues become empty (Soft kill switch)
  std::thread thread;

  ConcurrentVector<std::pair<double, cv::Mat>> input_image_queue;
  ConcurrentVector<Eigen::Matrix<double, 7, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 7, 1>>> input_imu_queue;
  ConcurrentVector<PreprocessedFrame::Ptr> input_frame_queue;

  ConcurrentVector<EstimationFrame::ConstPtr> output_estimation_results;
  ConcurrentVector<EstimationFrame::ConstPtr> output_marginalized_frames;

  std::shared_ptr<OdometryEstimationBase> odometry_estimation;
};

}  // namespace glim