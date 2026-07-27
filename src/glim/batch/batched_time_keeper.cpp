#include <glim/batch/batched_time_keeper.hpp>

#include <iostream>
#include <spdlog/spdlog.h>
#include <gtsam_points/util/parallelism.hpp>
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
  const auto per_thread_task = [this, &batched_imu](int i) {
    ScopedCallbackContext ctx(i);
    for (int j = 0; j < (*batched_imu)[i].size(); j++) {
      if (!std::isfinite((*batched_imu)[i][j](0))) {
        continue;
      }

      if (!time_keepers[i]->validate_imu_stamp((*batched_imu)[i][j](0))) {
        std::cerr << "Warning: IMU data at batch index " << i << ", measurement index " << j << " is invalid and will be skipped." << std::endl;
      }
    }
  };

  if (gtsam_points::is_tbb_default()) {
#ifdef GTSAM_POINTS_USE_TBB
    tbb::parallel_for(0, batched_imu->size(), 1, per_thread_task);
    return;
#else
    spdlog::error("BatchedIMUMeasurements: TBB is enabled but gtsam_points was built without TBB support!!");
    abort();
#endif
  }

#pragma omp parallel for num_threads(num_threads)
  for (int i = 0; i < batched_imu->size(); i++) {
    per_thread_task(i);
  }
}

void BatchedTimeKeeper::process(const BatchedRawPoints::Ptr& batched_points) {
  const auto per_thread_task = [this, &batched_points](int i) {
    ScopedCallbackContext ctx(i);
    time_keepers[i]->process((*batched_points)[i]);
  };

  if (gtsam_points::is_tbb_default()) {
#ifdef GTSAM_POINTS_USE_TBB
    tbb::parallel_for(0, batched_points->size(), 1, per_thread_task);
    return;
#else
    spdlog::error("BatchedRawPoints: TBB is enabled but gtsam_points was built without TBB support!!");
    abort();
#endif
  }

#pragma omp parallel for schedule(dynamic) num_threads(num_threads)
  for (int i = 0; i < batched_points->size(); i++) {
    per_thread_task(i);
  }
}

}  // namespace glim
