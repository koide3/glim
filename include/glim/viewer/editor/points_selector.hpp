#pragma once

#include <spdlog/spdlog.h>
#include <gtsam_points/util/vector3i_hash.hpp>
#include <gtsam_points/segmentation/min_cut.hpp>
#include <gtsam_points/segmentation/region_growing.hpp>
#include <glim/viewer/editor/map_cell.hpp>

namespace glim {

struct PointsSelector {
public:
  PointsSelector(std::shared_ptr<spdlog::logger> logger);
  ~PointsSelector();

  void clear();
  void set_submaps(const std::vector<glim::SubMap::Ptr>& submaps);
  void update_cells();

  void draw_ui();

private:
  float map_cell_resolution;
  int cell_selection_window;

  std::vector<glim::SubMap::Ptr> submaps;
  std::unordered_map<Eigen::Vector3i, MapCell::Ptr, gtsam_points::Vector3iHash> map_cells;

  Eigen::Vector3d picked_point;
  std::vector<MapCell::Ptr> selected_cells;

  //
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