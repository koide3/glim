#pragma once

#include <random>
#include <vector>
#include <Eigen/Core>
#include <glim/preprocess/preprocessed_frame.hpp>

namespace glim {

PreprocessedFrame::Ptr downsample(const std::vector<double>& times, const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& points, double resolution);

PreprocessedFrame::Ptr downsample_randomgrid(
  const std::vector<double>& times,
  const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& points,
  std::mt19937& mt,
  double resolution,
  double sampling_rate);
}