#pragma once

#include <spdlog/spdlog.h>
#include <gtsam_points/util/vector3i_hash.hpp>
#include <gtsam_points/segmentation/min_cut.hpp>
#include <gtsam_points/segmentation/region_growing.hpp>
#include <glim/viewer/editor/map_cell.hpp>

#include <glk/pointcloud_buffer.hpp>

namespace guik {
class ModelControl;
}

namespace glim {

struct PointsSelector {
public:
  PointsSelector(std::shared_ptr<spdlog::logger> logger);
  ~PointsSelector();

  void clear();
  void set_submaps(const std::vector<glim::SubMap::Ptr>& submaps);

  void draw_ui();

private:
  gtsam_points::PointCloudCPU::Ptr collect_submap_points(const std::vector<std::uint64_t>& point_ids);

  void update_cells();
  void remove_points();

  void select_points_tool();
  void select_points_radius();
  void select_points_segmentation();

private:
  // Map data
  std::vector<SubMap::Ptr> submaps;
  std::vector<glk::PointCloudBuffer::Ptr> submap_drawables;

  std::unordered_map<SubMap::ConstPtr, std::unordered_set<MapCell::Ptr>> cell_map;
  std::unordered_map<Eigen::Vector3i, MapCell::Ptr, gtsam_points::Vector3iHash> map_cells;

  // Selection data
  std::vector<std::uint64_t> selected_point_ids;     // (submap_id << 32bit) | point_id
  gtsam_points::PointCloudCPU::Ptr selected_points;  // all points in the selected cells

  Eigen::Vector3d picked_point;
  std::unique_ptr<guik::ModelControl> model_control;

  float map_cell_resolution;
  int cell_selection_window;

  bool show_picked_point;
  bool show_selection_radius;

  bool show_gizmo;
  int selected_tool;
  float select_radius;

  bool show_segmentation_radius;
  int segmentation_method;
  gtsam_points::MinCutParams min_cut_params;
  gtsam_points::RegionGrowingParams region_growing_params;

  // logging
  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim