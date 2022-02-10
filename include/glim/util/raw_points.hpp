#pragma once

#include <memory>
#include <vector>
#include <Eigen/Core>

namespace glim {

struct RawPoints {
public:
  using Ptr = std::shared_ptr<RawPoints>;
  using ConstPtr = std::shared_ptr<const RawPoints>;

  int size() const { return points.size(); }

public:
  double stamp;
  std::vector<double> times;
  std::vector<double> intensities;
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> points;
};

}  // namespace glim
