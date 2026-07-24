#pragma once

#include <memory>
#include <glim/util/raw_points.hpp>

namespace glim {

class BatchedRawPoints {
public:
  using Ptr = std::shared_ptr<BatchedRawPoints>;
  using ConstPtr = std::shared_ptr<const BatchedRawPoints>;

  BatchedRawPoints(int batch_size);
  ~BatchedRawPoints();

  int size() const;

  RawPoints::Ptr& operator[](int index);
  const RawPoints::ConstPtr operator[](int index) const;

private:
  std::vector<RawPoints::Ptr> raw_points;
};

}  // namespace glim