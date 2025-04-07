#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/preprocess/callbacks.hpp>

#include <fstream>
#include <iostream>
#include <spdlog/spdlog.h>
#include <gtsam_points/config.hpp>
#include <gtsam_points/ann/kdtree.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/util/parallelism.hpp>

#include <glim/util/config.hpp>
#include <glim/util/convert_to_string.hpp>

#ifdef GTSAM_POINTS_USE_TBB
#include <tbb/task_arena.h>
#include <tbb/parallel_for.h>
#endif

namespace glim {

CloudPreprocessorParams::CloudPreprocessorParams() {
  Config config(GlobalConfig::get_config_path("config_preprocess"));
  Config sensor_config(GlobalConfig::get_config_path("config_sensors"));

  global_shutter = sensor_config.param<bool>("sensors", "global_shutter_lidar", false);

  distance_near_thresh = config.param<double>("preprocess", "distance_near_thresh", 1.0);
  distance_far_thresh = config.param<double>("preprocess", "distance_far_thresh", 100.0);
  use_random_grid_downsampling = config.param<bool>("preprocess", "use_random_grid_downsampling", false);
  downsample_resolution = config.param<double>("preprocess", "downsample_resolution", 0.15);
  downsample_target = config.param<int>("preprocess", "random_downsample_target", 0);
  downsample_rate = config.param<double>("preprocess", "random_downsample_rate", 0.3);
  enable_outlier_removal = config.param<bool>("preprocess", "enable_outlier_removal", false);
  outlier_removal_k = config.param<int>("preprocess", "outlier_removal_k", 10);
  outlier_std_mul_factor = config.param<double>("preprocess", "outlier_std_mul_factor", 2.0);

  enable_cropbox_filter = config.param<bool>("preprocess", "enable_cropbox_filter", false);
  crop_bbox_frame = config.param<std::string>("preprocess", "crop_bbox_frame", "lidar");
  crop_bbox_min = config.param<Eigen::Vector3d>("preprocess", "crop_bbox_min", {});
  crop_bbox_max = config.param<Eigen::Vector3d>("preprocess", "crop_bbox_max", {});
  Eigen::Isometry3d T_lidar_imu = sensor_config.param<Eigen::Isometry3d>("sensors", "T_lidar_imu", Eigen::Isometry3d::Identity());
  T_imu_lidar = T_lidar_imu.inverse();

  if (enable_cropbox_filter) {
    if (crop_bbox_frame != "lidar" && crop_bbox_frame != "imu") {
      throw std::runtime_error(fmt::format("Unsupported crop bbox frame: {}", crop_bbox_frame));
    } else if ((crop_bbox_min.array() > crop_bbox_max.array()).any()) {
      throw std::runtime_error(fmt::format("Misconfigured bbox: min={}, max={}", convert_to_string(crop_bbox_min), convert_to_string(crop_bbox_max)));
    }
  }

  k_correspondences = config.param<int>("preprocess", "k_correspondences", 8);

  num_threads = config.param<int>("preprocess", "num_threads", 2);
}

CloudPreprocessorParams::~CloudPreprocessorParams() {}

CloudPreprocessor::CloudPreprocessor(const CloudPreprocessorParams& params) : params(params) {
#ifdef GTSAM_POINTS_USE_TBB
  if (gtsam_points::is_tbb_default()) {
    tbb_task_arena.reset(new tbb::task_arena(params.num_threads));
  }
#endif
}

CloudPreprocessor::~CloudPreprocessor() {}

PreprocessedFrame::Ptr CloudPreprocessor::preprocess(const RawPoints::ConstPtr& raw_points) {
  PreprocessCallbacks::on_raw_points_received(raw_points);
  if (gtsam_points::is_omp_default() || params.num_threads == 1 || !tbb_task_arena) {
    return preprocess_impl(raw_points);
  }

  PreprocessedFrame::Ptr preprocessed;
#ifdef GTSAM_POINTS_USE_TBB
  auto arena = static_cast<tbb::task_arena*>(tbb_task_arena.get());
  arena->execute([&] { preprocessed = preprocess_impl(raw_points); });
#else
  std::cerr << "error : TBB is not enabled" << std::endl;
  abort();
#endif
  return preprocessed;
}

PreprocessedFrame::Ptr CloudPreprocessor::preprocess_impl(const RawPoints::ConstPtr& raw_points) {
  spdlog::trace("preprocessing input: {} points", raw_points->size());

  gtsam_points::PointCloud::Ptr frame(new gtsam_points::PointCloud);
  frame->num_points = raw_points->size();
  frame->times = const_cast<double*>(raw_points->times.data());
  frame->points = const_cast<Eigen::Vector4d*>(raw_points->points.data());
  if (raw_points->intensities.size()) {
    frame->intensities = const_cast<double*>(raw_points->intensities.data());
  }

  // Downsampling
  if (params.use_random_grid_downsampling) {
    const double rate = params.downsample_target > 0 ? static_cast<double>(params.downsample_target) / frame->size() : params.downsample_rate;
    frame = gtsam_points::randomgrid_sampling(frame, params.downsample_resolution, rate, mt, params.num_threads);
  } else {
    frame = gtsam_points::voxelgrid_sampling(frame, params.downsample_resolution, params.num_threads);
  }

  if (frame->size() < 100) {
    spdlog::warn("too few points in the downsampled cloud ({} points)", frame->size());
  }

  // Distance filter
  std::vector<int> indices;
  indices.reserve(frame->size());
  double squared_distance_near_thresh = params.distance_near_thresh * params.distance_near_thresh;
  double squared_distance_far_thresh  = params.distance_far_thresh  * params.distance_far_thresh;

  for (int i = 0; i < frame->size(); i++) {
    const bool is_finite = frame->points[i].allFinite();
    const double squared_dist = (Eigen::Vector4d() << frame->points[i].head<3>(), 0.0).finished().squaredNorm();
    if (squared_dist > squared_distance_near_thresh && squared_dist < squared_distance_far_thresh && is_finite) {
      indices.push_back(i);
    }
  }

  if (indices.size() < 100) {
    spdlog::warn("too few points in the filtered cloud ({} points)", indices.size());
  }

  // Sort by time
  std::sort(indices.begin(), indices.end(), [&](const int lhs, const int rhs) { return frame->times[lhs] < frame->times[rhs]; });
  frame = gtsam_points::sample(frame, indices);

  if (params.global_shutter) {
    std::fill(frame->times, frame->times + frame->size(), 0.0);
  }

  // Cropbox filter
  if (params.enable_cropbox_filter) {
    if (params.crop_bbox_frame == "lidar") {
      auto is_inside_bbox = [&](const Eigen::Vector3d& p_lidar) {
        return (p_lidar.array() >= params.crop_bbox_min.array()).all()
            && (p_lidar.array() <= params.crop_bbox_max.array()).all();
      };

      frame = gtsam_points::filter(frame, [&](const auto& pt) {
        return !is_inside_bbox(pt.template head<3>());
      });

    } else if (params.crop_bbox_frame == "imu") {
      auto is_inside_bbox = [&](const Eigen::Vector3d& p_lidar) {
        const auto p_imu = params.T_imu_lidar * p_lidar;
        return (p_imu.array() >= params.crop_bbox_min.array()).all()
            && (p_imu.array() <= params.crop_bbox_max.array()).all();
      };

      frame = gtsam_points::filter(frame, [&](const auto& pt) {
        return !is_inside_bbox(pt.template head<3>());
      });

    } else {
      throw std::runtime_error(fmt::format("Unsupported crop bbox frame: {}", params.crop_bbox_frame));
    }
  }

  // Outlier removal
  if (params.enable_outlier_removal) {
    frame = gtsam_points::remove_outliers(frame, params.outlier_removal_k, params.outlier_std_mul_factor, params.num_threads);
  }

  // Create a preprocessed frame
  PreprocessedFrame::Ptr preprocessed(new PreprocessedFrame);
  preprocessed->stamp = raw_points->stamp;
  preprocessed->scan_end_time = frame->size() ? raw_points->stamp + frame->times[frame->size() - 1] : raw_points->stamp;

  preprocessed->times.assign(frame->times, frame->times + frame->size());
  preprocessed->points.assign(frame->points, frame->points + frame->size());
  if (frame->intensities) {
    preprocessed->intensities.assign(frame->intensities, frame->intensities + frame->size());
  }

  preprocessed->k_neighbors = params.k_correspondences;
  preprocessed->neighbors = find_neighbors(frame->points, frame->size(), params.k_correspondences);

  spdlog::trace("preprocessed: {} -> {} points", raw_points->size(), preprocessed->size());

  return preprocessed;
}

std::vector<int> CloudPreprocessor::find_neighbors(const Eigen::Vector4d* points, const int num_points, const int k) const {
  gtsam_points::KdTree tree(points, num_points);

  std::vector<int> neighbors(num_points * k);

  const auto perpoint_task = [&](int i) {
    std::vector<size_t> k_indices(k);
    std::vector<double> k_sq_dists(k);
    tree.knn_search(points[i].data(), k, k_indices.data(), k_sq_dists.data());
    std::copy(k_indices.begin(), k_indices.end(), neighbors.begin() + i * k);
  };

  if (gtsam_points::is_omp_default()) {
#pragma omp parallel for num_threads(params.num_threads) schedule(guided, 8)
    for (int i = 0; i < num_points; i++) {
      perpoint_task(i);
    }
  } else {
#ifdef GTSAM_POINTS_USE_TBB
    tbb::parallel_for(tbb::blocked_range<int>(0, num_points, 8), [&](const tbb::blocked_range<int>& range) {
      for (int i = range.begin(); i < range.end(); i++) {
        perpoint_task(i);
      }
    });
#else
    std::cerr << "error : TBB is not enabled" << std::endl;
    abort();
#endif
  }

  return neighbors;
}

}  // namespace glim
