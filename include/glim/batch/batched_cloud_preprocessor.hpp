#pragma once

#include <memory>
#include <glim/batch/batched_raw_points.hpp>
#include <glim/batch/batched_preprocessed_frame.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>

namespace glim {

class BatchedCloudPreprocessor {
public:
  BatchedCloudPreprocessor(int batch_size, int num_threads = -1, const CloudPreprocessorParams& params_ = CloudPreprocessorParams());

  BatchedPreprocessedFrame::Ptr process(const BatchedRawPoints::Ptr& batched_points);

private:
  int num_threads;
  std::vector<std::shared_ptr<CloudPreprocessor>> preprocessors;
};

}  // namespace glim