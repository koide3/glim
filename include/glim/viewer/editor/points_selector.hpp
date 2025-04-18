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

/// @brief Interactive point selection
struct PointsSelector {
public:
  PointsSelector(std::shared_ptr<spdlog::logger> logger);
  ~PointsSelector();

  void clear();
  void set_submaps(const std::vector<glim::SubMap::Ptr>& submaps);

  void draw_ui();

private:
  std::vector<std::uint64_t> collect_neighbor_point_ids(const Eigen::Vector3d& point);
  gtsam_points::PointCloudCPU::Ptr collect_submap_points(const std::vector<std::uint64_t>& point_ids);

  void update_cells();
  void remove_selected_points();

  void select_points_tool();
  void select_points_radius();
  void select_outlier_points_radius();
  void select_points_segmentation();

private:
  // Map data
  std::vector<SubMap::Ptr> submaps;                          ///< Submaps
  std::vector<glk::PointCloudBuffer::Ptr> submap_drawables;  ///< Submap cloud buffers

  std::unordered_map<SubMap::ConstPtr, std::unordered_set<MapCell::Ptr>> cell_map;          ///< submap -> cell indices
  std::unordered_map<Eigen::Vector3i, MapCell::Ptr, gtsam_points::Vector3iHash> map_cells;  ///< Map cells for fast neighbor query

  // Selection data
  std::vector<std::uint64_t> selected_point_ids;  ///< Selected points (submap_id << 32bit) | point_id

  Eigen::Vector3d picked_point;                       ///< 3D point picked by a right click
  std::unique_ptr<guik::ModelControl> model_control;  ///< Gizmo for manipulating the selection tools

  int num_threads;            ///< Number of threads
  float map_cell_resolution;  ///< Map cell resolution [m]
  int cell_selection_window;  ///< Cell selection window size [m]

  bool show_preferences_window;

  bool show_picked_point;
  bool show_selection_radius;

  bool show_cells;
  bool show_gizmo;
  int selected_tool;

  float select_radius;
  float outlier_radius_offset;
  int outliner_num_neighbors;
  float outlier_stddev_thresh;

  bool show_segmentation_radius;
  int segmentation_method;
  gtsam_points::MinCutParams min_cut_params;
  gtsam_points::RegionGrowingParams region_growing_params;

  // logging
  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim