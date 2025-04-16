#pragma once

#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <glim/mapping/sub_map.hpp>
#include <glk/pointcloud_buffer.hpp>

namespace glim {

struct MapCell {
public:
  using Ptr = std::shared_ptr<MapCell>;
  using ConstPtr = std::shared_ptr<const MapCell>;

  MapCell(double resolution, const Eigen::Vector3i& coord);
  ~MapCell();

  std::string name() const;

  void add_point(const SubMap::Ptr& submap, int point_id);
  void finalize();
  void reset_points();

public:
  const double resolution;
  const Eigen::Vector3i coord;

  std::vector<glim::SubMap::Ptr> submaps;
  glk::PointCloudBuffer::Ptr cloud_buffer;

  std::vector<std::uint64_t> point_ids;
  gtsam_points::PointCloudCPU::Ptr points;
};

}  // namespace glim
