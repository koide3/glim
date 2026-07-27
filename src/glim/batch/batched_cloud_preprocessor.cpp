#include <glim/batch/batched_cloud_preprocessor.hpp>

#include <spdlog/spdlog.h>
#include <gtsam_points/util/parallelism.hpp>
#include <glim/util/callback_slot.hpp>

#ifdef GTSAM_POINTS_USE_TBB
#include <tbb/parallel_for.h>
#endif

namespace glim {

BatchedCloudPreprocessor::BatchedCloudPreprocessor(int batch_size, int num_threads, const CloudPreprocessorParams& params_) {
#ifdef _OPENMP
  this->num_threads = (num_threads > 0) ? num_threads : omp_get_max_threads();
#else
  this->num_threads = 1;
#endif

  CloudPreprocessorParams params = params_;
  params.num_threads = 1;  // Disable multithreading in each preprocessor to avoid oversubscription

  preprocessors.resize(batch_size);
  for (int i = 0; i < batch_size; i++) {
    ScopedCallbackContext ctx(i);
    preprocessors[i] = std::make_shared<CloudPreprocessor>(params);
  }
}

BatchedPreprocessedFrame::Ptr BatchedCloudPreprocessor::process(const BatchedRawPoints::Ptr& batched_points) {
  auto batched_frame = std::make_shared<BatchedPreprocessedFrame>(batched_points->size());

  const auto per_thread_task = [this, &batched_points, &batched_frame](int i) {
    ScopedCallbackContext ctx(i);
    (*batched_frame)[i] = preprocessors[i]->preprocess((*batched_points)[i]);
  };

  if (gtsam_points::is_tbb_default()) {
#ifdef GTSAM_POINTS_USE_TBB
    tbb::parallel_for(0, batched_points->size(), 1, per_thread_task);
    return batched_frame;
#else
    spdlog::error("BatchedPreprocessedFrame: TBB is enabled but gtsam_points was built without TBB support!!");
    abort();
#endif
  }

#pragma omp parallel for schedule(dynamic) num_threads(num_threads)
  for (int i = 0; i < batched_points->size(); i++) {
    per_thread_task(i);
  }
  return batched_frame;
}

}  // namespace glim
