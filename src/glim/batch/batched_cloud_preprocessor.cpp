#include <glim/batch/batched_cloud_preprocessor.hpp>
#include <glim/util/callback_slot.hpp>

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

#pragma omp parallel for schedule(dynamic) num_threads(num_threads)
  for (int i = 0; i < batched_points->size(); i++) {
    ScopedCallbackContext ctx(i);
    (*batched_frame)[i] = preprocessors[i]->preprocess((*batched_points)[i]);
  }

  return batched_frame;
}

}  // namespace glim
