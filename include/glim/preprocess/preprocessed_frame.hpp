#pragma once

#include <memory>
#include <vector>
#include <Eigen/Core>
#include <glim/util/raw_points.hpp>

namespace glim {

/**
 * @brief Preprocessed point cloud
 *
 */
struct PreprocessedFrame {
public:
  using Ptr = std::shared_ptr<PreprocessedFrame>;
  using ConstPtr = std::shared_ptr<const PreprocessedFrame>;

  /**
   * @brief Number of points
   * @return Number of points
   */
  int size() const { return points.size(); }

public:
  double stamp;          // Timestamp at the beginning of the scan
  double scan_end_time;  // Timestamp at the end of the scan

  std::vector<double> times;            // Point timestamps w.r.t. the first pt
  std::vector<double> intensities;      // Point intensities
  std::vector<Eigen::Vector4d> points;  // Points (homogeneous coordinates)

  int k_neighbors;             // Number of neighbors of each point
  std::vector<int> neighbors;  // k-nearest neighbors of each point

  RawPoints::ConstPtr raw_points;
};

}  // namespace glim