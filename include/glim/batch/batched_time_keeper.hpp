#pragma once

#include <memory>
#include <glim/util/time_keeper.hpp>
#include <glim/batch/batched_imu_measurements.hpp>
#include <glim/batch/batched_raw_points.hpp>

namespace glim {

class BatchedTimeKeeper {
public:
  using Ptr = std::shared_ptr<BatchedTimeKeeper>;
  using ConstPtr = std::shared_ptr<const BatchedTimeKeeper>;

  BatchedTimeKeeper(int batch_size, int num_threads = -1);
  ~BatchedTimeKeeper();

  void validate_imu_stamp(const BatchedIMUMeasurements::Ptr& batched_imu);
  void process(const BatchedRawPoints::Ptr& batched_points);

private:
  int num_threads;
  std::vector<std::shared_ptr<TimeKeeper>> time_keepers;
};

}  // namespace glim