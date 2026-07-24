#pragma once

#include <Eigen/Core>
#include <glim/odometry/estimation_frame.hpp>

namespace glim {

class BatchedEstimationFrame {
public:
  using Ptr = std::shared_ptr<BatchedEstimationFrame>;
  using ConstPtr = std::shared_ptr<const BatchedEstimationFrame>;

  BatchedEstimationFrame(int batch_size);
  ~BatchedEstimationFrame();

  int size() const;

  EstimationFrame::Ptr& operator[](int index);
  const EstimationFrame::ConstPtr operator[](int index) const;

private:
  std::vector<EstimationFrame::Ptr> frames;
};

}  // namespace glim