#include <glim/batch/batched_time_keeper.hpp>

#include <iostream>
#include <glim/util/callback_slot.hpp>

namespace glim {

BatchedTimeKeeper::BatchedTimeKeeper(int batch_size, int num_threads) {
#ifdef _OPENMP
  this->num_threads = (num_threads > 0) ? num_threads : omp_get_max_threads();
#else
  this->num_threads = 1;
#endif

  time_keepers.reserve(batch_size);
  for (int i = 0; i < batch_size; i++) {
    ScopedCallbackContext ctx(i);
    time_keepers.emplace_back(std::make_shared<TimeKeeper>());
  }
}

BatchedTimeKeeper::~BatchedTimeKeeper() {}

void BatchedTimeKeeper::validate_imu_stamp(const BatchedIMUMeasurements::Ptr& batched_imu) {
#pragma omp parallel for num_threads(num_threads)
  for (int i = 0; i < batched_imu->size(); i++) {
    ScopedCallbackContext ctx(i);
    for (int j = 0; j < (*batched_imu)[i].size(); j++) {
      if (!std::isfinite((*batched_imu)[i][j](0))) {
        continue;
      }

      if (!time_keepers[i]->validate_imu_stamp((*batched_imu)[i][j](0))) {
        std::cerr << "Warning: IMU data at batch index " << i << ", measurement index " << j << " is invalid and will be skipped." << std::endl;
      }
    }
  }
}

void BatchedTimeKeeper::process(const BatchedRawPoints::Ptr& batched_points) {
#pragma omp parallel for schedule(dynamic) num_threads(num_threads)
  for (int i = 0; i < batched_points->size(); i++) {
    ScopedCallbackContext ctx(i);
    time_keepers[i]->process((*batched_points)[i]);
  }
}

}  // namespace glim
