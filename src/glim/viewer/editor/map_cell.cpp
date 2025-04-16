#include <glim/viewer/editor/map_cell.hpp>

#include <fmt/format.h>

namespace glim {

MapCell::MapCell(double resolution, const Eigen::Vector3i& coord) : resolution(resolution), coord(coord) {}

MapCell::~MapCell() {}

std::string MapCell::name() const {
  return fmt::format("cell_{}_{}_{}", coord[0], coord[1], coord[2]);
}

void MapCell::clear() {
  submaps.clear();
  point_ids.clear();
}

void MapCell::add_point(const SubMap::Ptr& submap, int point_id) {
  if (std::find(submaps.begin(), submaps.end(), submap) == submaps.end()) {
    submaps.emplace_back(submap);
  }

  point_ids.emplace_back((static_cast<std::uint64_t>(submap->id) << 32) | point_id);
}

void MapCell::remove_submap(int submap_id) {
  const auto found = std::find_if(submaps.begin(), submaps.end(), [=](const SubMap::Ptr& submap) { return submap->id == submap_id; });
  if (found != submaps.end()) {
    submaps.erase(found);
  }

  const auto remove_loc = std::remove_if(point_ids.begin(), point_ids.end(), [=](std::uint64_t id) { return (id >> 32) == submap_id; });
  point_ids.erase(remove_loc, point_ids.end());
}

}  // namespace glim
