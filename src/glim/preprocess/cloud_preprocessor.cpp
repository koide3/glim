#include <glim/preprocess/cloud_preprocessor.hpp>

#include <fstream>
#include <iostream>
#include <gtsam_ext/ann/kdtree.hpp>

#include <gtsam_ext/types/frame_cpu.hpp>

#include <glim/util/config.hpp>
#include <glim/util/console_colors.hpp>

namespace glim {

CloudPreprocessorParams::CloudPreprocessorParams() {
  Config config(GlobalConfig::get_config_path("config_preprocess"));
  Config sensor_config(GlobalConfig::get_config_path("config_sensors"));

  T_lidar_offset.setIdentity();
  auto lidar_offset = sensor_config.param<Eigen::Isometry3d>("sensors", "T_lidar_offset");
  if (lidar_offset) {
    T_lidar_offset = *lidar_offset;
  }

  use_random_grid_downsampling = config.param<bool>("preprocess", "use_random_grid_downsampling", false);

  distance_near_thresh = config.param<double>("preprocess", "distance_near_thresh", 1.0);
  distance_far_thresh = config.param<double>("preprocess", "distance_far_thresh", 100.0);
  downsample_resolution = config.param<double>("preprocess", "downsample_resolution", 0.15);
  downsample_target = config.param<int>("preprocess", "random_downsample_target", 0);
  downsample_rate = config.param<double>("preprocess", "random_downsample_rate", 0.3);
  k_correspondences = config.param<int>("preprocess", "k_correspondences", 8);

  num_threads = config.param<int>("preprocess", "num_threads", 10);
}

CloudPreprocessorParams::~CloudPreprocessorParams() {}

CloudPreprocessor::CloudPreprocessor(const CloudPreprocessorParams& params) : params(params) {}

CloudPreprocessor::~CloudPreprocessor() {}

PreprocessedFrame::Ptr CloudPreprocessor::preprocess(const RawPoints::ConstPtr& raw_points) {
  gtsam_ext::Frame::Ptr frame(new gtsam_ext::Frame);
  frame->num_points = raw_points->size();
  frame->times = const_cast<double*>(raw_points->times.data());
  frame->points = const_cast<Eigen::Vector4d*>(raw_points->points.data());
  if (raw_points->intensities.size()) {
    frame->intensities = const_cast<double*>(raw_points->intensities.data());
  }

  // Downsampling
  if (params.use_random_grid_downsampling) {
    const double rate = params.downsample_target > 0 ? static_cast<double>(params.downsample_target) / frame->size() : params.downsample_rate;
    frame = gtsam_ext::randomgrid_sampling(frame, params.downsample_resolution, rate, mt);
  } else {
    frame = gtsam_ext::voxelgrid_sampling(frame, params.downsample_resolution);
  }

  // Distance filter
  std::vector<int> indices;
  indices.reserve(frame->size());
  for (int i = 0; i < frame->size(); i++) {
    const double dist = (Eigen::Vector4d() << frame->points[i].head<3>(), 0.0).finished().norm();
    if (dist > params.distance_near_thresh && dist < params.distance_far_thresh) {
      indices.push_back(i);
    }
  }

  // Sort by time
  std::sort(indices.begin(), indices.end(), [&](const int lhs, const int rhs) { return frame->times[lhs] < frame->times[rhs]; });
  frame = gtsam_ext::sample(frame, indices);

  // Create a preprocessed frame
  PreprocessedFrame::Ptr preprocessed(new PreprocessedFrame);
  preprocessed->stamp = raw_points->stamp;
  preprocessed->scan_end_time = raw_points->stamp + frame->times[frame->size() - 1];
  preprocessed->times.assign(frame->times, frame->times + frame->size());
  preprocessed->points.assign(frame->points, frame->points + frame->size());
  preprocessed->intensities.assign(frame->intensities, frame->intensities + frame->size());

  // Nearest neighbor search
  preprocessed->k_neighbors = params.k_correspondences;
  preprocessed->neighbors = find_neighbors(preprocessed->points, params.k_correspondences);

  return preprocessed;
}

std::vector<int> CloudPreprocessor::find_neighbors(const Points& points, int k) const {
  gtsam_ext::KdTree tree(points.data(), points.size());

  std::vector<int> neighbors(points.size() * k);

#pragma omp parallel for num_threads(params.num_threads) schedule(guided, 8)
  for (int i = 0; i < points.size(); i++) {
    std::vector<size_t> k_indices(k);
    std::vector<double> k_sq_dists(k);
    tree.knn_search(points[i].data(), k, k_indices.data(), k_sq_dists.data());
    std::copy(k_indices.begin(), k_indices.end(), neighbors.begin() + i * k);
  }

  return neighbors;
}

}  // namespace glim