#pragma once

#include <spdlog/spdlog.h>
#include <glk/drawable.hpp>
#include <guik/progress_modal.hpp>
#include <glim/mapping/sub_map.hpp>

namespace glim {

class SubMapEditor;

class MapEditor {
public:
  MapEditor(const std::string& init_map_path);
  ~MapEditor();

  void run();

private:
  void update_viewer();

  void main_menu();

  void ui_callback();

  std::vector<glim::SubMap::Ptr> load_submaps(guik::ProgressInterface& progress, const std::string& map_path);

  bool save_submaps(guik::ProgressInterface& progress, const std::string& save_path);

private:
  std::shared_ptr<spdlog::logger> logger;
  std::unique_ptr<guik::ProgressModal> progress_modal;

  std::string map_path;
  std::string init_map_path;

  int selected_submap;
  std::vector<glim::SubMap::Ptr> submaps;
  std::vector<glk::Drawable::Ptr> submap_drawables;

  std::unique_ptr<SubMapEditor> submap_editor;
};

}  // namespace glim
