#pragma once

#include <random>
#include <vector>
#include <Eigen/Core>

#include <glim/preprocess/preprocessed_frame.hpp>

namespace glim {

class CloudPreprocessor {
public:
  using Points = std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>;

  CloudPreprocessor();
  virtual ~CloudPreprocessor();

  virtual PreprocessedFrame::Ptr preprocess(double stamp, const std::vector<double>& times, const Points& points) const;

public:
  PreprocessedFrame::Ptr sort_by_time(const std::vector<double>& times, const Points& points) const;
  PreprocessedFrame::Ptr distance_filter(const std::vector<double>& times, const Points& points) const;
  std::vector<int> find_neighbors(const Points& points, int k) const;

private:
  bool use_random_grid_downsampling;
  double distance_near_thresh;
  double distance_far_thresh;
  double downsample_resolution;
  double downsample_rate;
  int k_correspondences;

  mutable std::mt19937 mt;
};

}  // namespace glim