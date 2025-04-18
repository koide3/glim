#include <glim/viewer/map_editor.hpp>

#include <fstream>
#include <iostream>
#include <filesystem>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/ringbuffer_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <portable-file-dialogs.h>
#include <guik/spdlog_sink.hpp>
#include <guik/recent_files.hpp>
#include <guik/model_control.hpp>
#include <guik/progress_modal.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <glk/drawable_container.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/indexed_pointcloud_buffer.hpp>
#include <gtsam_points/ann/kdtree2.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/features/normal_estimation.hpp>
#include <gtsam_points/segmentation/min_cut.hpp>
#include <gtsam_points/segmentation/region_growing.hpp>
#include <gtsam_points/util/fast_floor.hpp>
#include <GLFW/glfw3.h>

#include <glim/mapping/sub_map.hpp>
#include <glim/util/logging.hpp>

namespace glim {

////////////////////////////////////////////////////////////////////////
MapEditor::MapEditor(const std::string& init_map_path) : init_map_path(init_map_path) {
  logger = get_default_logger();

  auto viewer = guik::viewer();

  viewer->register_ui_callback("log", guik::create_logger_ui(get_ringbuffer_sink(), 0.8));
  viewer->register_ui_callback("ui_callback", [this] { ui_callback(); });

  progress_modal.reset(new guik::ProgressModal("progress"));

  selector.reset(new PointsSelector(logger));
}

MapEditor::~MapEditor() {}

void MapEditor::run() {
  guik::viewer()->spin();
}

void MapEditor::main_menu() {
  bool start_open_map = false;
  bool start_save_map = false;

  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      // open map
      if (ImGui::MenuItem("Open New Map")) {
        if (!submaps.empty()) {
          if (pfd::message("Warning", "Close the current map?").result() == pfd::button::ok) {
            submaps.clear();
            selector->clear();
            start_open_map = true;
          }
        } else {
          start_open_map = true;
        }
      }

      // save map
      if (ImGui::MenuItem("Save map")) {
        if (submaps.empty()) {
          logger->warn("No map to save");
        } else {
          start_save_map = true;
        }
      }

      // close map
      if (ImGui::MenuItem("Close map")) {
        if (submaps.empty()) {
          logger->warn("No map to close");
        } else if (pfd::message("Warning", "Close the map?").result() == pfd::button::ok) {
          submaps.clear();
          selector->clear();
        }
      }

      ImGui::EndMenu();
    }

    ImGui::EndMainMenuBar();
  }

  // Map open modal
  if (start_open_map || !init_map_path.empty()) {
    guik::RecentFiles recent_files("offline_viewer_open");
    if (init_map_path.empty()) {
      // Open a dialog to select a map path
      map_path = pfd::select_folder("Select a dump directory", recent_files.most_recent()).result();
    } else {
      // If the map path is given as a command line argument, use it
      map_path = init_map_path;
      init_map_path.clear();
    }

    if (!map_path.empty()) {
      logger->info("Load map from {}", map_path);
      recent_files.push(map_path);

      // Start map loading
      progress_modal->open<std::vector<glim::SubMap::Ptr>>("open", [this](guik::ProgressInterface& progress) { return load_submaps(progress, map_path); });
    }
  }

  // Catch the loaded submaps
  const auto loaded_submaps = progress_modal->run<std::vector<glim::SubMap::Ptr>>("open");
  if (loaded_submaps && !loaded_submaps->empty()) {
    submaps = *loaded_submaps;
    selector->set_submaps(submaps);
  }

  // Map save modal
  if (start_save_map) {
    guik::RecentFiles recent_files("offline_viewer_save");
    const std::string path = pfd::select_folder("Select a save directory", recent_files.most_recent()).result();
    if (!path.empty()) {
      recent_files.push(path);
      progress_modal->open<bool>("save", [this, path](guik::ProgressInterface& progress) { return save_submaps(progress, path); });
    }
  }

  const auto save_result = progress_modal->run<bool>("save");
  if (save_result) {
    if (*save_result) {
      logger->info("Map saved");
    } else {
      logger->warn("Failed to save map");
    }
  }
}

void MapEditor::ui_callback() {
  auto viewer = guik::viewer();

  main_menu();

  selector->draw_ui();
}

std::vector<glim::SubMap::Ptr> MapEditor::load_submaps(guik::ProgressInterface& progress, const std::string& map_path) {
  progress.set_title("Load submaps");
  progress.set_text("Now loading");
  std::ifstream ifs(map_path + "/graph.txt");
  if (!ifs) {
    logger->warn("Failed to open {}/graph.txt", map_path);
    return {};
  }

  std::string token;
  int num_submaps;
  ifs >> token >> num_submaps;

  if (token != "num_submaps:") {
    logger->warn("Invalid graph.txt format");
    return {};
  }

  progress.set_maximum(num_submaps);

  std::vector<glim::SubMap::Ptr> submaps(num_submaps);
  for (int i = 0; i < num_submaps; i++) {
    progress.increment();
    submaps[i] = glim::SubMap::load(fmt::format("{}/{:06d}", map_path, i));
    if (!submaps[i]) {
      logger->warn("Failed to load submap {}", i);
      return {};
    }

    if (!submaps[i]->frame->has_normals()) {
      auto frame = std::dynamic_pointer_cast<gtsam_points::PointCloudCPU>(submaps[i]->frame);
      if (!frame) {
        logger->warn("Failed to cast frame to PointCloudCPU");
      } else {
        frame->add_normals(gtsam_points::estimate_normals(frame->points, frame->covs, frame->size(), 4));
      }
    }
  }

  logger->info("loaded {} submaps", num_submaps);

  return submaps;
}

bool MapEditor::save_submaps(guik::ProgressInterface& progress, const std::string& save_path) {
  progress.set_title("Save map");
  progress.set_maximum(submaps.size());

  logger->info("Saving {} submaps to {}", submaps.size(), map_path);

  if (save_path != this->map_path) {
    logger->info("Copying metadata from {} to {}", map_path, save_path);
    progress.set_text(fmt::format("Copying metadata from {} to {}", map_path, save_path));
    std::filesystem::copy(
      std::filesystem::path(map_path),
      std::filesystem::path(save_path),
      std::filesystem::copy_options::recursive | std::filesystem::copy_options::overwrite_existing);
  }

  for (size_t i = 0; i < submaps.size(); i++) {
    progress.set_title(fmt::format("Save submap {}/{}", i, submaps.size()));
    progress.increment();
    submaps[i]->save(fmt::format("{}/{:06d}", save_path, i));
  }

  return true;
}

}  // namespace glim