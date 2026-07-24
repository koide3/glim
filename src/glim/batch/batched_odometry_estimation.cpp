#include <glim/batch/batched_odometry_estimation.hpp>
#include <glim/util/callback_slot.hpp>

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
  const int batch_size = batched_imu->size();
  for (int i = 0; i < batch_size; i++) {
    ScopedCallbackContext ctx(i);

    const auto& imu = (*batched_imu)[i];
    for (const auto& imu_measurement : imu) {
      odometries[i]->insert_imu(imu_measurement(0), imu_measurement.segment<3>(1), imu_measurement.segment<3>(4));
    }
  }
}

BatchedEstimationFrame::Ptr BatchedOdometryEstimation::insert_frame(
  const BatchedPreprocessedFrame::Ptr& batched_frame,
  std::vector<std::vector<EstimationFrame::ConstPtr>>& marginalized_states) {
  const int batch_size = batched_frame->size();
  auto batched_estimation_frame = std::make_shared<BatchedEstimationFrame>(batch_size);
  marginalized_states.resize(batch_size);

#pragma omp parallel for schedule(dynamic) num_threads(num_threads)
  for (int i = 0; i < batch_size; i++) {
    ScopedCallbackContext ctx(i);
    (*batched_estimation_frame)[i] = std::const_pointer_cast<EstimationFrame>(odometries[i]->insert_frame((*batched_frame)[i], marginalized_states[i]));
  }

  return batched_estimation_frame;
}

}  // namespace glim
