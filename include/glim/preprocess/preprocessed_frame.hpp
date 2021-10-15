#pragma once

#include <memory>
#include <vector>
#include <Eigen/Core>

namespace glim {

struct PreprocessedFrame {
public:
  using Ptr = std::shared_ptr<PreprocessedFrame>;
  using ConstPtr = std::shared_ptr<const PreprocessedFrame>;

  int size() const { return points.size(); }

public:
  double stamp;
  double scan_end_time;

  std::vector<double> times;
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> points;

  int k_neighbors;
  std::vector<int> neighbors;
};

}  // namespace glim