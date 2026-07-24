#include <glim/batch/batched_raw_points.hpp>

namespace glim {

BatchedRawPoints::BatchedRawPoints(int batch_size) : raw_points(batch_size) {
  for (auto& points : raw_points) {
    points = std::make_shared<RawPoints>();
  }
}

BatchedRawPoints::~BatchedRawPoints() {}

int BatchedRawPoints::size() const {
  return raw_points.size();
}

RawPoints::Ptr& BatchedRawPoints::operator[](int index) {
  return raw_points[index];
}

const RawPoints::ConstPtr BatchedRawPoints::operator[](int index) const {
  return raw_points[index];
}

}  // namespace glim
