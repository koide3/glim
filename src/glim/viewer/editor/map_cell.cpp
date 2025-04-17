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
  if (std::find(submaps.begin(), submaps.end(), submap.get()) == submaps.end()) {
    submaps.emplace_back(submap.get());
  }

  point_ids.emplace_back((static_cast<std::uint64_t>(submap->id) << 32) | point_id);
}

void MapCell::add_points(const SubMap::Ptr& submap, const std::vector<int>& point_ids) {
  if (std::find(submaps.begin(), submaps.end(), submap.get()) == submaps.end()) {
    submaps.emplace_back(submap.get());
  }

  for (const auto& point_id : point_ids) {
    this->point_ids.emplace_back((static_cast<std::uint64_t>(submap->id) << 32) | point_id);
  }
}

void MapCell::remove_submap(int submap_id) {
  const auto found = std::find_if(submaps.begin(), submaps.end(), [=](const SubMap* submap) { return submap->id == submap_id; });
  if (found != submaps.end()) {
    submaps.erase(found);
  }

  const auto remove_loc = std::remove_if(point_ids.begin(), point_ids.end(), [=](std::uint64_t id) { return (id >> 32) == submap_id; });
  point_ids.erase(remove_loc, point_ids.end());
}

void MapCell::remove_submaps(const std::vector<int>& submap_ids) {
  const auto if_remove = [=](int submap_id) { return std::find(submap_ids.begin(), submap_ids.end(), submap_id) != submap_ids.end(); };

  const auto remove_loc = std::remove_if(submaps.begin(), submaps.end(), [=](const SubMap* submap) { return if_remove(submap->id); });
  submaps.erase(remove_loc, submaps.end());

  const auto remove_loc2 = std::remove_if(point_ids.begin(), point_ids.end(), [=](std::uint64_t id) { return if_remove(id >> 32); });
  point_ids.erase(remove_loc2, point_ids.end());
}

}  // namespace glim
