#include <random>
#include <Eigen/Core>
#include <boost/functional/hash.hpp>

#include <glim/preprocess/preprocessed_frame.hpp>

namespace glim {

class Vector3iHash {
public:
  size_t operator()(const Eigen::Vector3i& x) const {
    size_t seed = 0;
    boost::hash_combine(seed, x[0]);
    boost::hash_combine(seed, x[1]);
    boost::hash_combine(seed, x[2]);
    return seed;
  }
};

struct AveragingVoxel {
public:
  using Ptr = std::shared_ptr<AveragingVoxel>;

  AveragingVoxel(double time_thresh) : time_thresh(time_thresh) {}
  ~AveragingVoxel() {}

  void add(double time, const Eigen::Vector4d& point) {
    for (int i = 0; i < num_points.size(); i++) {
      if (std::abs(sum_times[i] / num_points[i] - time) > time_thresh) {
        continue;
      }

      num_points[i]++;
      sum_times[i] += time;
      sum_points[i] += point;
      return;
    }

    num_points.push_back(1);
    sum_times.push_back(time);
    sum_points.push_back(point);
  }

  void finalize(std::vector<double>& times, std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& points) {
    for (int i = 0; i < num_points.size(); i++) {
      times.push_back(sum_times[i] / num_points[i]);
      points.push_back(sum_points[i] / num_points[i]);
    }
  }

  const double time_thresh;
  std::vector<int> num_points;
  std::vector<double> sum_times;
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> sum_points;
};

PreprocessedFrame::Ptr downsample(const std::vector<double>& times, const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& points, double resolution) {
  using VoxelMap = std::unordered_map<
    Eigen::Vector3i,
    AveragingVoxel::Ptr,
    Vector3iHash,
    std::equal_to<Eigen::Vector3i>,
    Eigen::aligned_allocator<std::pair<const Eigen::Vector3i, AveragingVoxel::Ptr>>>;

  VoxelMap voxels;

  const double scan_duration = times.back() - times.front();
  const double time_thresh = scan_duration / 5;

  for (int i = 0; i < times.size(); i++) {
    Eigen::Vector3i coord = (points[i].array() / resolution - 0.5).floor().cast<int>().head<3>();
    auto found = voxels.find(coord);
    if (found == voxels.end()) {
      found = voxels.insert(found, std::make_pair(coord, std::make_shared<AveragingVoxel>(time_thresh)));
    }
    found->second->add(times[i], points[i]);
  }

  PreprocessedFrame::Ptr downsampled(new PreprocessedFrame());
  downsampled->times.reserve(2 * voxels.size());
  downsampled->points.reserve(2 * voxels.size());

  for (const auto& voxel : voxels) {
    voxel.second->finalize(downsampled->times, downsampled->points);
  }

  return downsampled;
}

struct RandomSamplingVoxel {
public:
  using Ptr = std::shared_ptr<RandomSamplingVoxel>;

  RandomSamplingVoxel() {}
  ~RandomSamplingVoxel() {}

  int size() const { return indices.size(); }

  void add(int index) { indices.push_back(index); }

  std::vector<int> indices;
};

PreprocessedFrame::Ptr downsample_randomgrid(
  const std::vector<double>& times,
  const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& points,
  std::mt19937& mt,
  double resolution,
  double sampling_rate) {
  using VoxelMap = std::unordered_map<
    Eigen::Vector3i,
    RandomSamplingVoxel::Ptr,
    Vector3iHash,
    std::equal_to<Eigen::Vector3i>,
    Eigen::aligned_allocator<std::pair<const Eigen::Vector3i, RandomSamplingVoxel::Ptr>>>;

  VoxelMap voxelmap;

  for (int i = 0; i < times.size(); i++) {
    Eigen::Vector3i coord = (points[i].array() / resolution - 0.5).floor().cast<int>().head<3>();
    auto found = voxelmap.find(coord);
    if (found == voxelmap.end()) {
      found = voxelmap.insert(found, std::make_pair(coord, std::make_shared<RandomSamplingVoxel>()));
    }
    found->second->add(i);
  }

  const int points_per_voxel = std::ceil((sampling_rate * points.size()) / voxelmap.size());

  PreprocessedFrame::Ptr downsampled(new PreprocessedFrame);
  downsampled->times.reserve(times.size() * sampling_rate * 1.2);
  downsampled->points.reserve(points.size() * sampling_rate * 1.2);

  for (const auto& voxel : voxelmap) {
    const auto& indices = voxel.second->indices;
    if (indices.size() <= points_per_voxel) {
      for (const int index : indices) {
        downsampled->times.push_back(times[index]);
        downsampled->points.push_back(points[index]);
      }
    } else {
      std::vector<int> sampled(points_per_voxel);
      std::sample(indices.begin(), indices.end(), sampled.begin(), points_per_voxel, mt);
      for (const int index : sampled) {
        downsampled->times.push_back(times[index]);
        downsampled->points.push_back(points[index]);
      }
    }
  }

  return downsampled;
}

}  // namespace glim