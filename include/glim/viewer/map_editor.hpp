#pragma once

#include <unordered_map>

#include <spdlog/spdlog.h>
#include <guik/progress_modal.hpp>
#include <glim/mapping/sub_map.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim/viewer/editor/map_cell.hpp>
#include <glim/viewer/editor/points_selector.hpp>

namespace glim {

/// @brief Interactive map editor
class MapEditor {
public:
  MapEditor(const std::string& init_map_path);
  ~MapEditor();

  void run();

private:
  void main_menu();

  void ui_callback();

  std::vector<glim::SubMap::Ptr> load_submaps(guik::ProgressInterface& progress, const std::string& map_path);

  bool save_submaps(guik::ProgressInterface& progress, const std::string& save_path);

private:
  std::shared_ptr<spdlog::logger> logger;
  std::unique_ptr<guik::ProgressModal> progress_modal;

  std::string map_path;       ///< Input map path
  std::string init_map_path;  ///< Map path for initial auto loading (if path is given as a command line argument)

  std::vector<glim::SubMap::Ptr> submaps;    ///< Submaps
  std::unique_ptr<PointsSelector> selector;  ///< Points selector
};

}  // namespace glim