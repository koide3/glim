#pragma once

#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <glim/mapping/sub_map.hpp>

namespace glim {

/// @brief A cell to store submap point indices for quick indexing.
struct MapCell {
public:
  using Ptr = std::shared_ptr<MapCell>;
  using ConstPtr = std::shared_ptr<const MapCell>;

  MapCell(double resolution, const Eigen::Vector3i& coord);
  ~MapCell();

  std::string name() const;

  void clear();
  void add_point(int submap_id, int point_id);
  void add_points(int submap_id, const std::vector<int>& point_ids);

  void remove_submap(int submap_id);
  void remove_submaps(const std::vector<int>& submap_ids);

public:
  const double resolution;      // Cell resolution [m]
  const Eigen::Vector3i coord;  // Cell coordinates in the map frame

  std::vector<std::uint64_t> point_ids;  // (submap_id << 32bit) | point_id
};

}  // namespace glim
