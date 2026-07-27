#include <glim/batch/batched_odometry_estimation.hpp>

#include <spdlog/spdlog.h>
#include <gtsam_points/util/parallelism.hpp>
#include <glim/util/callback_slot.hpp>

#ifdef GTSAM_POINTS_USE_TBB
#include <tbb/parallel_for.h>
#endif

namespace glim {

BatchedOdometryEstimation::BatchedOdometryEstimation(int batch_size, const std::string& so_name, int num_threads) : odometries(batch_size) {
#ifdef _OPENMP
  this->num_threads = (num_threads > 0) ? num_threads : omp_get_max_threads();
#else
  this->num_threads = 1;
#endif

  for (int i = 0; i < batch_size; i++) {
    ScopedCallbackContext ctx(i);
    odometries[i] = OdometryEstimationBase::load_module(so_name);
  }
}

void BatchedOdometryEstimation::insert_imu(const BatchedIMUMeasurements::Ptr& batched_imu) {
  const auto per_thread_task = [this, &batched_imu](int i) {
    ScopedCallbackContext ctx(i);
    const auto& imu = (*batched_imu)[i];
    for (const auto& imu_measurement : imu) {
      odometries[i]->insert_imu(imu_measurement(0), imu_measurement.segment<3>(1), imu_measurement.segment<3>(4));
    }
  };

#ifdef GTSAM_POINTS_USE_TBB
  if (gtsam_points::is_tbb_default()) {
    tbb::parallel_for(0, batched_imu->size(), 1, per_thread_task);
    return;
  } else {
    spdlog::error("BatchedIMUMeasurements: TBB is disabled!!");
    abort();
  }
#endif

  for (int i = 0; i < batched_imu->size(); i++) {
    per_thread_task(i);
  }
}

BatchedEstimationFrame::Ptr BatchedOdometryEstimation::insert_frame(
  const BatchedPreprocessedFrame::Ptr& batched_frame,
  std::vector<std::vector<EstimationFrame::ConstPtr>>& marginalized_states) {
  auto batched_estimation_frame = std::make_shared<BatchedEstimationFrame>(batched_frame->size());
  marginalized_states.resize(batched_frame->size());

  const auto per_thread_task = [this, &batched_frame, &batched_estimation_frame, &marginalized_states](int i) {
    ScopedCallbackContext ctx(i);
    (*batched_estimation_frame)[i] = std::const_pointer_cast<EstimationFrame>(odometries[i]->insert_frame((*batched_frame)[i], marginalized_states[i]));
  };

  if (gtsam_points::is_tbb_default()) {
#ifdef GTSAM_POINTS_USE_TBB
    tbb::parallel_for(0, batched_frame->size(), 1, per_thread_task);
    return batched_estimation_frame;
#else
    spdlog::error("BatchedEstimationFrame: TBB is enabled but gtsam_points was built without TBB support!!");
    abort();
#endif
  }

#pragma omp parallel for schedule(dynamic) num_threads(num_threads)
  for (int i = 0; i < batched_frame->size(); i++) {
    per_thread_task(i);
  }

  return batched_estimation_frame;
}

}  // namespace glim
