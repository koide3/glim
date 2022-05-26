#pragma once

#include <memory>
#include <vector>
#include <Eigen/Core>

namespace glim {

/**
 * @brief Raw point cloud frame
 */
struct RawPoints {
public:
  using Ptr = std::shared_ptr<RawPoints>;
  using ConstPtr = std::shared_ptr<const RawPoints>;

  /// Number of points
  int size() const { return points.size(); }

public:
  double stamp;                                                                     ///< Timestamp of the first point
  std::vector<double> times;                                                        ///< Per-point timestamps relative to the first point
  std::vector<double> intensities;                                                  ///< Point intensities
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> points;   ///< Point coordinates
};

}  // namespace glim
