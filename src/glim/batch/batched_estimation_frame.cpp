#include <glim/batch/batched_estimation_frame.hpp>

namespace glim {

BatchedEstimationFrame::BatchedEstimationFrame(int batch_size) : frames(batch_size) {}

BatchedEstimationFrame::~BatchedEstimationFrame() {}

int BatchedEstimationFrame::size() const {
  return frames.size();
}

EstimationFrame::Ptr& BatchedEstimationFrame::operator[](int index) {
  return frames[index];
}

const EstimationFrame::ConstPtr BatchedEstimationFrame::operator[](int index) const {
  return frames[index];
}

}  // namespace glim
