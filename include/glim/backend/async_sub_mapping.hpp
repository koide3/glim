#pragma once

#include <atomic>
#include <thread>
#include <glim/backend/sub_mapping.hpp>
#include <glim/util/concurrent_vector.hpp>

namespace glim {

class AsyncSubMapping {
public:
  AsyncSubMapping(const std::shared_ptr<glim::SubMappingBase>& sub_mapping);
  ~AsyncSubMapping();

  void insert_image(const double stamp, const cv::Mat& image);
  void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);
  void insert_frame(const EstimationFrame::ConstPtr& odom_frame);

  void join();

  int input_queue_size() const;
  int output_queue_size() const;

  std::vector<SubMap::Ptr> get_results();

private:
  void run();

private:
  std::atomic_bool kill_switch;      // Flag to stop the thread immediately (Hard kill switch)
  std::atomic_bool end_of_sequence;  // Flag to stop the thread when the input queues become empty (Soft kill switch)
  std::thread thread;

  ConcurrentVector<std::pair<double, cv::Mat>> input_image_queue;
  ConcurrentVector<Eigen::Matrix<double, 7, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 7, 1>>> input_imu_queue;
  ConcurrentVector<EstimationFrame::ConstPtr> input_frame_queue;

  ConcurrentVector<SubMap::Ptr> output_submap_queue;

  std::shared_ptr<glim::SubMappingBase> sub_mapping;
};

}  // namespace  glim
