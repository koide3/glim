#pragma once

#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <glim/mapping/sub_map.hpp>

namespace glim {

struct MapCell {
public:
  using Ptr = std::shared_ptr<MapCell>;
  using ConstPtr = std::shared_ptr<const MapCell>;

  MapCell(double resolution, const Eigen::Vector3i& coord);
  ~MapCell();

  std::string name() const;

  void clear();
  void add_point(const SubMap::Ptr& submap, int point_id);
  void add_points(const SubMap::Ptr& submap, const std::vector<int>& point_ids);

  void remove_submap(int submap_id);
  void remove_submaps(const std::vector<int>& submap_ids);

public:
  const double resolution;
  const Eigen::Vector3i coord;

  std::vector<glim::SubMap*> submaps;

  std::vector<std::uint64_t> point_ids;  // (submap_id << 32bit) | point_id
};

}  // namespace glim
