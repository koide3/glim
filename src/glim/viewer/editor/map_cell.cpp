#include <glim/viewer/editor/map_cell.hpp>

#include <fmt/format.h>

namespace glim {

MapCell::MapCell(double resolution, const Eigen::Vector3i& coord) : resolution(resolution), coord(coord), points(new gtsam_points::PointCloudCPU) {}

MapCell::~MapCell() {}

std::string MapCell::name() const {
  return fmt::format("cell_{}_{}_{}", coord[0], coord[1], coord[2]);
}

void MapCell::add_point(const SubMap::Ptr& submap, int point_id) {
  if (std::find(submaps.begin(), submaps.end(), submap) == submaps.end()) {
    submaps.emplace_back(submap);
  }
  point_ids.emplace_back((static_cast<std::uint64_t>(submap->id) << 32) | point_id);

  points->points_storage.emplace_back(submap->T_world_origin * submap->frame->points[point_id]);
  if (submap->frame->has_covs()) {
    const Eigen::Matrix4d T = submap->T_world_origin.matrix();
    points->covs_storage.emplace_back(T * submap->frame->covs[point_id] * T.transpose());
  }
  if (submap->frame->has_normals()) {
    points->normals_storage.emplace_back(submap->T_world_origin.matrix() * submap->frame->normals[point_id]);
  }
}

void MapCell::finalize() {
  points->points = points->points_storage.data();
  points->covs = points->covs_storage.data();
  points->normals = points->normals_storage.data();
  points->num_points = points->points_storage.size();

  cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points->points, points->num_points);
}

void MapCell::reset_points() {
  point_ids.clear();
  points.reset(new gtsam_points::PointCloudCPU);

  const Eigen::Array4d min_pt = Eigen::Array4d(coord[0], coord[1], coord[2], 0).cast<double>() * resolution;
  const Eigen::Array4d max_pt = min_pt + Eigen::Array4d(resolution, resolution, resolution, 2.0);

  for (const auto& submap : submaps) {
    for (int i = 0; i < submap->frame->size(); i++) {
      const Eigen::Array4d pt = submap->T_world_origin * submap->frame->points[i];
      if ((pt < min_pt).any() || (pt > max_pt).any()) {
        continue;
      }

      add_point(submap, i);
    }
  }

  finalize();
}

}  // namespace glim
