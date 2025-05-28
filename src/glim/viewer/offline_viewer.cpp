#include <glim/viewer/offline_viewer.hpp>

#include <boost/filesystem.hpp>
#include <gtsam_points/config.hpp>
#include <gtsam_points/optimizers/linearization_hook.hpp>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu_create.hpp>
#include <glim/util/config.hpp>

#include <spdlog/spdlog.h>
#include <portable-file-dialogs.h>
#include <glk/io/ply_io.hpp>
#include <guik/recent_files.hpp>
#include <guik/progress_modal.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace glim {

OfflineViewer::OfflineViewer(const std::string& init_map_path) : init_map_path(init_map_path) {}

OfflineViewer::~OfflineViewer() {}

void OfflineViewer::setup_ui() {
  auto viewer = guik::LightViewer::instance();
  viewer->register_ui_callback("main_menu", [this] { main_menu(); });

  progress_modal.reset(new guik::ProgressModal("offline_viewer_progress"));

#ifdef GTSAM_POINTS_USE_CUDA
  gtsam_points::LinearizationHook::register_hook([] { return gtsam_points::create_nonlinear_factor_set_gpu(); });
#endif
}

void OfflineViewer::main_menu() {
  bool start_open_map = false || !init_map_path.empty();
  bool start_close_map = false;
  bool start_save_map = false;
  bool start_export_map = false;

  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      if (!async_global_mapping) {  // if a previously loaded map does not yet exist
        if (ImGui::MenuItem("Open New Map")) {
          start_open_map = true;
        }
      } else {
        if (ImGui::MenuItem("Open Additional Map")) {
          start_open_map = true;
        }
      }

      if (ImGui::MenuItem("Close Map")) {
        if (pfd::message("Warning", "Close the map?").result() == pfd::button::ok) {
          start_close_map = true;
        }
      }

      if (ImGui::BeginMenu("Save")) {
        if (ImGui::MenuItem("Save Map")) {
          start_save_map = true;
        }

        if (ImGui::MenuItem("Export Points")) {
          start_export_map = true;
        }

        ImGui::EndMenu();
      }

      if (ImGui::MenuItem("Quit")) {
        if (pfd::message("Warning", "Quit?").result() == pfd::button::ok) {
          request_to_terminate = true;
        }
      }

      ImGui::EndMenu();
    }

    ImGui::EndMainMenuBar();
  }

  // open map
  if (start_open_map) {
    logger->debug("open map");
    std::string map_path;

    guik::RecentFiles recent_files("offline_viewer_open");
    if (init_map_path.empty()) {
      map_path = pfd::select_folder("Select a dump directory", recent_files.most_recent()).result();
    } else {
      map_path = init_map_path;
      init_map_path.clear();
    }

    if (!map_path.empty()) {
      logger->debug("open map from {}", map_path);
      recent_files.push(map_path);

      if (boost::filesystem::exists(map_path + "/config")) {
        logger->info("Use config from {}", map_path + "/config");
        GlobalConfig::instance(map_path + "/config", true);
      } else {
        logger->warn("No config found in {}", map_path);
      }

      const Config config_ros(GlobalConfig::get_config_path("config_ros"));
      const std::vector<std::string> ext_module_names = config_ros.param<std::vector<std::string>>("glim_ros", "extension_modules", {});
      for (const auto& name : ext_module_names) {
        if (name.find("viewer") != std::string::npos || name.find("monitor") != std::string::npos) {
          continue;
        }
        if (imported_shared_libs.count(name)) {
          logger->debug("Extension module {} already loaded", name);
          continue;
        }

        logger->info("Export classes from {}", name);
        ExtensionModule::export_classes(name);
        imported_shared_libs.insert(name);
      }

      // if a map is already loaded, use existing map to load new map into
      std::shared_ptr<GlobalMapping> global_mapping;
      if (async_global_mapping) {
        logger->info("global map already exists, loading new map into existing global map");
        global_mapping = std::dynamic_pointer_cast<GlobalMapping>(async_global_mapping->get_global_mapping());
      }

      progress_modal->open<std::shared_ptr<GlobalMapping>>("open", [this, map_path, global_mapping](guik::ProgressInterface& progress) {
        return load_map(progress, map_path, global_mapping);
      });
    }
  }
  auto open_result = progress_modal->run<std::shared_ptr<GlobalMapping>>("open");
  if (open_result) {
    if (!(*open_result)) {
      pfd::message("Error", "Failed to load map").result();
    } else {
      async_global_mapping.reset(new glim::AsyncGlobalMapping(*open_result, 1e6));
    }
  }

  // save map
  if (start_save_map) {
    if (!async_global_mapping) {
      logger->warn("No map data to save");
    } else {
      guik::RecentFiles recent_files("offline_viewer_save");
      const std::string path = pfd::select_folder("Select a directory to save the map", recent_files.most_recent()).result();
      if (!path.empty()) {
        recent_files.push(path);
        progress_modal->open<bool>("save", [this, path](guik::ProgressInterface& progress) { return save_map(progress, path); });
      }
    }
  }
  auto save_result = progress_modal->run<bool>("save");

  // export map
  if (start_export_map) {
    guik::RecentFiles recent_files("offline_viewer_export");
    const std::string path = pfd::save_file("Select the file destination", recent_files.most_recent(), {"PLY", "*.ply"}).result();
    if (!path.empty()) {
      recent_files.push(path);
      progress_modal->open<bool>("export", [this, path](guik::ProgressInterface& progress) { return export_map(progress, path); });
    }
  }
  auto export_result = progress_modal->run<bool>("export");

  // close map
  if (start_close_map) {
    if (async_global_mapping) {
      logger->info("Closing map");
      async_global_mapping->join();
      async_global_mapping.reset();
      clear();
    } else {
      logger->warn("No map to close");
    }
  }
}

std::shared_ptr<glim::GlobalMapping> OfflineViewer::load_map(guik::ProgressInterface& progress, const std::string& path, std::shared_ptr<GlobalMapping> global_mapping) {
  progress.set_title("Load map");
  progress.set_text("Now loading");
  progress.set_maximum(1);

  if (global_mapping == nullptr) {  // if no map is loaded yet initialize new GlobalMapping
    glim::GlobalMappingParams params;
    params.isam2_relinearize_skip = 1;
    params.isam2_relinearize_thresh = 0.0;

    const auto result = pfd::message("Confirm", "Do optimization?", pfd::choice::yes_no).result();
    params.enable_optimization = (result == pfd::button::ok) || (result == pfd::button::yes);

    logger->info("enable_optimization={}", params.enable_optimization);
    global_mapping.reset(new glim::GlobalMapping(params));
  }

  if (!global_mapping->load(path)) {
    logger->error("failed to load {}", path);
    return nullptr;
  }

  return global_mapping;
}

bool OfflineViewer::save_map(guik::ProgressInterface& progress, const std::string& path) {
  progress.set_title("Save map");
  progress.set_text("Now saving");
  async_global_mapping->save(path);
  return true;
}

bool OfflineViewer::export_map(guik::ProgressInterface& progress, const std::string& path) {
  progress.set_title("Export points");
  progress.set_text("Concatenating submaps");
  progress.set_maximum(3);
  progress.increment();
  const auto points = async_global_mapping->export_points();

  progress.set_text("Writing to file");
  progress.increment();
  glk::save_ply_binary(path, points.data(), points.size());

  return true;
}

}  // namespace glim