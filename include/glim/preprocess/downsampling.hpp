#pragma once

#include <random>
#include <vector>
#include <Eigen/Core>
#include <glim/preprocess/preprocessed_frame.hpp>

namespace glim {

/**
 * @brief Voxelgrid downsampling
 *
 * @param times       Timestamps of points
 * @param points      Points to be downsampled
 * @param intensities Point intensities
 * @param resolution  Voxel resolution
 * @return PreprocessedFrame::Ptr Downsampled points
 */
PreprocessedFrame::Ptr downsample(
  const std::vector<double>& times,
  const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& points,
  const std::vector<double>& intensities,
  double resolution);

PreprocessedFrame::Ptr downsample(const std::vector<double>& times, const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& points, double resolution);

/**
 * @brief Voxelgrid random downsampling for non-uniform scan LiDARs (e.g., Livox)
 *
 * @param times           Timestamps of points
 * @param points          Points to be downsampled
 * @param mt              RNG
 * @param resolution      Voxel resolution
 * @param sampling_rate   Random sampling rate (num_output_points = sampling_rate * num_input_points)
 * @return PreprocessedFrame::Ptr Downsampled points
 */
PreprocessedFrame::Ptr downsample_randomgrid(
  const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& points,
  const std::vector<double>& intensities,
  std::mt19937& mt,
  double resolution,
  double sampling_rate);

PreprocessedFrame::Ptr downsample_randomgrid(
  const std::vector<double>& times,
  const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& points,
  const std::vector<double>& intensities,
  std::mt19937& mt,
  double resolution,
  double sampling_rate);

PreprocessedFrame::Ptr downsample_randomgrid(
  const std::vector<double>& times,
  const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& points,
  std::mt19937& mt,
  double resolution,
  double sampling_rate);
}