#include <glim/preprocess/cloud_preprocessor.hpp>

#include <iostream>
#include <gtsam_ext/ann/kdtree.hpp>

#include <glim/util/config.hpp>
#include <glim/util/console_colors.hpp>
#include <glim/preprocess/downsampling.hpp>

namespace glim {

CloudPreprocessor::CloudPreprocessor() {
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
  downsample_rate = config.param<double>("preprocess", "random_downsample_rate", 0.3);
  k_correspondences = config.param<int>("preprocess", "k_correspondences", 8);
}

CloudPreprocessor::~CloudPreprocessor() {}

PreprocessedFrame::Ptr CloudPreprocessor::preprocess(double stamp, const std::vector<double>& times, const Points& points) const {
  PreprocessedFrame::Ptr frame(new PreprocessedFrame);
  frame->times = times;
  frame->points.resize(points.size());
  std::transform(points.begin(), points.end(), frame->points.begin(), [this](const Eigen::Vector4d& p) { return T_lidar_offset * p; });

  frame = distance_filter(frame->times, frame->points);
  if (use_random_grid_downsampling) {
    frame = downsample_randomgrid(frame->times, frame->points, mt, downsample_resolution, downsample_rate);
  } else {
    frame = downsample(frame->times, frame->points, downsample_resolution);
  }
  frame = sort_by_time(frame->times, frame->points);

  frame->stamp = stamp;
  frame->scan_end_time = stamp + frame->times.back();
  frame->k_neighbors = k_correspondences;
  frame->neighbors = find_neighbors(frame->points, k_correspondences);

  return frame;
}

PreprocessedFrame::Ptr CloudPreprocessor::sort_by_time(const std::vector<double>& times, const Points& points) const {
  // sort by time
  std::vector<int> indices(times.size());
  std::iota(indices.begin(), indices.end(), 0);
  std::sort(indices.begin(), indices.end(), [&](const int& lhs, const int& rhs) { return times[lhs] < times[rhs]; });

  PreprocessedFrame::Ptr sorted(new PreprocessedFrame());
  sorted->times.resize(times.size());
  sorted->points.resize(points.size());
  for (int i = 0; i < times.size(); i++) {
    const int index = indices[i];
    sorted->times[i] = times[index];
    sorted->points[i] = points[index];
  }

  return sorted;
}

PreprocessedFrame::Ptr CloudPreprocessor::distance_filter(const std::vector<double>& times, const Points& points) const {
  PreprocessedFrame::Ptr filtered(new PreprocessedFrame());
  filtered->times.reserve(times.size());
  filtered->points.reserve(points.size());

  for (int i = 0; i < points.size(); i++) {
    const double dist = points[i].norm();
    if (!std::isfinite(dist)) {
      std::cout << console::yellow << "warning: an invalid point found!! " << points[i].transpose() << console::reset << std::endl;
      continue;
    }

    if (dist < distance_near_thresh || dist > distance_far_thresh) {
      continue;
    }

    filtered->times.push_back(times[i]);
    filtered->points.push_back(points[i]);
  }

  return filtered;
}

std::vector<int> CloudPreprocessor::find_neighbors(const Points& points, int k) const {
  gtsam_ext::KdTree tree(points.data(), points.size());

  std::vector<int> neighbors(points.size() * k);

  std::vector<size_t> k_indices(k);
  std::vector<double> k_sq_dists(k);
  for (int i = 0; i < points.size(); i++) {
    tree.knn_search(points[i].data(), k, k_indices.data(), k_sq_dists.data());
    std::copy(k_indices.begin(), k_indices.end(), neighbors.begin() + i * k);
  }

  return neighbors;
}

}  // namespace glim