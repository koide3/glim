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

}  // namespace glim
