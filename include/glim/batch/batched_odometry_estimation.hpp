#pragma once

#include <glim/odometry/odometry_estimation_base.hpp>
#include <glim/batch/batched_imu_measurements.hpp>
#include <glim/batch/batched_preprocessed_frame.hpp>
#include <glim/batch/batched_estimation_frame.hpp>

namespace glim {

class BatchedOdometryEstimation {
public:
  BatchedOdometryEstimation(int batch_size, const std::string& so_name, int num_threads = -1);

  void insert_imu(const BatchedIMUMeasurements::Ptr& batched_imu);

  BatchedEstimationFrame::Ptr insert_frame(const BatchedPreprocessedFrame::Ptr& batched_frame, std::vector<std::vector<EstimationFrame::ConstPtr>>& marginalized_states);

private:
  int num_threads;
  std::vector<std::shared_ptr<OdometryEstimationBase>> odometries;
};

}  // namespace glim