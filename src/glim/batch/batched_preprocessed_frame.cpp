#include <glim/batch/batched_preprocessed_frame.hpp>

namespace glim {

BatchedPreprocessedFrame::BatchedPreprocessedFrame(int batch_size) : frames(batch_size) {}

BatchedPreprocessedFrame::~BatchedPreprocessedFrame() {}

int BatchedPreprocessedFrame::size() const {
  return frames.size();
}

PreprocessedFrame::Ptr& BatchedPreprocessedFrame::operator[](int index) {
  return frames[index];
}

const PreprocessedFrame::ConstPtr BatchedPreprocessedFrame::operator[](int index) const {
  return frames[index];
}

void BatchedPreprocessedFrame::set_raw_points(const BatchedRawPoints::Ptr& batched_raw_points) {
  if (batched_raw_points->size() != frames.size()) {
    throw std::runtime_error("BatchedPreprocessedFrame::set_raw_points: size mismatch between batched_raw_points and frames");
  }

  for (int i = 0; i < frames.size(); i++) {
    frames[i]->raw_points = (*batched_raw_points)[i];
  }
}

}  // namespace glim
