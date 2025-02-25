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
#include <glk/pointcloud_buffer.hpp>
#include <glk/indexed_pointcloud_buffer.hpp>
#include <gtsam_points/ann/kdtree2.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/features/normal_estimation.hpp>
#include <gtsam_points/segmentation/min_cut.hpp>
#include <gtsam_points/segmentation/region_growing.hpp>
#include <GLFW/glfw3.h>

#include <glim/mapping/sub_map.hpp>
#include <glim/util/logging.hpp>

namespace glim {

class SubMapEditor {
public:
  SubMapEditor(const std::shared_ptr<spdlog::logger>& logger, int num_threads) : logger(logger) {
    show_selection_radius = false;
    draw_gizmo = true;
    selected_tool = 0;
    select_radius = 1.0f;
    picked_point.setZero();

    show_segmentation_radius = false;
    segmentation_method = 0;
    min_cut_params.num_threads = num_threads;
    region_growing_params.num_threads = num_threads;

    model_control.reset(new guik::ModelControl("model_control"));
    model_control->set_gizmo_operation("UNIVERSAL");
  }

  void draw_ui() {
    auto viewer = guik::viewer();

    const auto show_note = [](const std::string& text) {
      if (ImGui::IsItemHovered()) {
        ImGui::BeginTooltip();
        ImGui::Text("%s", text.c_str());
        ImGui::EndTooltip();
      }
      return false;
    };

    auto picked_point = viewer->pick_point(1, 3);
    if (picked_point) {
      this->picked_point = picked_point->cast<double>();
      ImGui::OpenPopup("context_menu");
    }

    show_selection_radius = false;
    show_segmentation_radius = false;

    if (ImGui::BeginPopup("context_menu", ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::Text("Point: (%.1f, %.1f, %.1f)", this->picked_point.x(), this->picked_point.y(), this->picked_point.z());
      ImGui::Separator();

      if (ImGui::MenuItem("Move gizmo here")) {
        Eigen::Matrix4f matrix = model_control->model_matrix();
        matrix.block<3, 1>(0, 3) = this->picked_point.cast<float>();
        model_control->set_model_matrix(matrix);
      }

      const bool radius_menu_opened = ImGui::BeginMenu("Select in radius");
      bool radius_selection_clicked = ImGui::IsItemClicked(ImGuiMouseButton_Left);

      if (radius_menu_opened) {
        show_selection_radius = true;
        ImGui::DragFloat("Radius", &select_radius, 0.01f, 0.01f, 100.0f);
        radius_selection_clicked |= ImGui::MenuItem("Select");
        show_note("Select points within the radius");
        ImGui::EndMenu();
      }
      if (radius_selection_clicked) {
        logger->info("Select in radius clicked");
        select_points_radius();
        ImGui::CloseCurrentPopup();
      }

      const bool segmentation_menu_opened = ImGui::BeginMenu("Segmentation");
      bool segmentation_clicked = ImGui::IsItemClicked(ImGuiMouseButton_Left);
      segmentation_clicked |= (ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S));

      if (segmentation_menu_opened) {
        show_segmentation_radius = true;
        ImGui::Combo("Method", &segmentation_method, "MinCut\0RegionGrowing\0");
        show_note(
          "Segmentation method\n"
          "- MinCut: Graph cut based segmentation for extracting an object in a radius\n"
          "- RegionGrowing: Region growing based segmentation for extracting a connected region");

        if (segmentation_method == 0) {
          float distance_sigma = min_cut_params.distance_sigma;
          float angle_sigma = min_cut_params.angle_sigma * 180.0 / M_PI;
          float foreground_mask_radius = min_cut_params.foreground_mask_radius;
          float background_mask_radius = min_cut_params.background_mask_radius;

          ImGui::DragFloat("Distance sigma", &distance_sigma, 0.01f, 0.01f, 10.0f);
          show_note("Disance proximity width [m]");
          ImGui::DragFloat("Angle sigma", &angle_sigma, 0.01f, 0.01f, 180.0f);
          show_note("Angle proximity width [deg]");
          ImGui::DragFloat("Foreground radius", &foreground_mask_radius, 0.01f, 0.01f, 100.0f);
          show_note("Points within this radius are marked as foreground");
          ImGui::DragFloat("Background radius", &background_mask_radius, 0.01f, 0.01f, 100.0f);
          show_note("Points out of this radius are marked as background");
          ImGui::DragInt("K neighbors", &min_cut_params.k_neighbors, 1, 1, 100);
          show_note("Number of neighbors for mincut graph contruction");

          min_cut_params.distance_sigma = distance_sigma;
          min_cut_params.angle_sigma = angle_sigma * M_PI / 180.0;
          min_cut_params.foreground_mask_radius = foreground_mask_radius;
          min_cut_params.background_mask_radius = background_mask_radius;

        } else if (segmentation_method == 1) {
          float distance_threshold = region_growing_params.distance_threshold;
          float angle_threshold = region_growing_params.angle_threshold * 180.0 / M_PI;
          float dilation_radius = region_growing_params.dilation_radius;

          ImGui::DragFloat("Distance threshold", &distance_threshold, 0.01f, 0.01f, 1.0f);
          show_note("Points connection distance threshold [m]");
          ImGui::DragFloat("Angle threshold", &angle_threshold, 0.01f, 0.01f, 180.0f);
          show_note("Points connection angle threshold [deg]");
          ImGui::DragFloat("Dilation radius", &dilation_radius, 0.01f, 0.01f, 100.0f);
          show_note("After performing region growing, the extracted region will be dilated by this radius to fill halls");

          region_growing_params.distance_threshold = distance_threshold;
          region_growing_params.angle_threshold = angle_threshold * M_PI / 180.0;
          region_growing_params.dilation_radius = dilation_radius;
        }

        segmentation_clicked |= ImGui::MenuItem("Segment", "Ctrl+S");

        ImGui::EndMenu();
      }

      if (segmentation_clicked) {
        logger->info("Segmentation clicked");
        select_points_segmentation();
        ImGui::CloseCurrentPopup();
      }

      ImGui::EndPopup();
    }

    if (!submap) {
      ImGui::Text("No submap selected");
    }

    // UI
    ImGui::Checkbox("##Draw_gizmo", &draw_gizmo);
    show_note("Draw gizmo for selection tool");

    ImGui::SameLine();
    if (ImGui::Button("Select points") || show_note("Select points within the selection tool")) {
      select_points_tool();
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    ImGui::Combo("Selection tool", &selected_tool, "Box\0Sphere\0");

    ImGui::Separator();
    if (ImGui::Button("Remove selected points") || (ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_R)) || show_note("Remove selected points from the submap")) {
      remove_points();
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(Ctrl+R)");

    if (draw_gizmo) {
      model_control->draw_gizmo();
    }

    // GL
    if (submap_drawable) {
      viewer->update_drawable("selected_submap", submap_drawable, guik::Rainbow());
    } else {
      viewer->remove_drawable("selected_submap");
    }

    if (selected_points.empty()) {
      viewer->remove_drawable("selected_points");
    }

    if (show_selection_radius) {
      viewer->update_wire_sphere("selection_radius", guik::FlatRed().translate(this->picked_point).scale(select_radius).set_alpha(0.5));
    } else {
      viewer->remove_drawable("selection_radius");
    }

    if (show_segmentation_radius && segmentation_method == 0) {
      viewer->update_wire_sphere("mincut_foreground_radius", guik::FlatRed().translate(this->picked_point).scale(min_cut_params.foreground_mask_radius).set_alpha(0.5));
      viewer->update_wire_sphere("mincut_background_radius", guik::FlatBlue().translate(this->picked_point).scale(min_cut_params.background_mask_radius).set_alpha(0.5));

    } else {
      viewer->remove_drawable("mincut_foreground_radius");
      viewer->remove_drawable("mincut_background_radius");
    }

    if (draw_gizmo) {
      switch (selected_tool) {
        case 0:  // Box
          viewer->update_cube("selection_tool", guik::FlatOrange(model_control->model_matrix()).set_alpha(0.5));
          break;
        case 1:  // Sphere
          viewer->update_sphere("selection_tool", guik::FlatOrange(model_control->model_matrix()).set_alpha(0.5));
          break;
      }
    } else {
      viewer->remove_drawable("selection_tool");
    }
  }

  void set_submap(const glim::SubMap::Ptr& submap) {
    if (submap == nullptr) {
      this->submap = nullptr;
      selected_points.clear();
      neg_selected_points.clear();
      return;
    }

    this->submap = submap;
    submap_drawable = std::make_shared<glk::PointCloudBuffer>(submap->frame->points, submap->frame->size());
    selected_points.clear();
    neg_selected_points.clear();
  }

  void select_points_tool() {
    const Eigen::Matrix4d inv_model_matrix = model_control->model_matrix().cast<double>().inverse();

    selected_points.clear();
    neg_selected_points.clear();
    for (int i = 0; i < submap->frame->size(); i++) {
      const Eigen::Vector3d pt = (inv_model_matrix * submap->frame->points[i]).head<3>();

      if (selected_tool == 0) {
        if ((pt.array() > Eigen::Array3d::Constant(-0.5)).all() && (pt.array() < Eigen::Array3d::Constant(0.5)).all()) {
          selected_points.emplace_back(i);
        } else {
          neg_selected_points.emplace_back(i);
        }
      } else {
        if (pt.norm() < 1.0) {
          selected_points.emplace_back(i);
        } else {
          neg_selected_points.emplace_back(i);
        }
      }
    }

    auto drawable = std::make_shared<glk::IndexedPointCloudBuffer>(submap_drawable, reinterpret_cast<unsigned int*>(selected_points.data()), selected_points.size());
    guik::viewer()->update_drawable("selected_points", drawable, guik::FlatOrange().set_point_scale(3.0));
  }

  void select_points_radius() {
    selected_points.clear();
    neg_selected_points.clear();

    for (int i = 0; i < submap->frame->size(); i++) {
      const double dist = (submap->frame->points[i].head<3>() - picked_point).norm();
      if (dist < select_radius) {
        selected_points.emplace_back(i);
      } else {
        neg_selected_points.emplace_back(i);
      }
    }

    auto drawable = std::make_shared<glk::IndexedPointCloudBuffer>(submap_drawable, reinterpret_cast<unsigned int*>(selected_points.data()), selected_points.size());
    guik::viewer()->update_drawable("selected_points", drawable, guik::FlatOrange().set_point_scale(3.0));
  }

  void select_points_segmentation() {
    selected_points.clear();
    neg_selected_points.clear();

    if (segmentation_method == 0) {
      std::vector<int> indices;
      const double sq_dist_thresh = std::pow(min_cut_params.background_mask_radius + 1.0, 2);
      for (size_t i = 0; i < submap->frame->size(); i++) {
        const double sq_dist = (submap->frame->points[i] - picked_point.homogeneous()).squaredNorm();
        if (sq_dist < sq_dist_thresh) {
          indices.push_back(i);
        }
      }

      auto filtered = gtsam_points::sample(submap->frame, indices);
      auto filtered_tree = std::make_shared<gtsam_points::KdTree2<gtsam_points::PointCloud>>(filtered);
      auto result = gtsam_points::min_cut(*filtered, *filtered_tree, picked_point.homogeneous(), min_cut_params);

      selected_points.resize(result.cluster_indices.size());
      std::transform(result.cluster_indices.begin(), result.cluster_indices.end(), selected_points.begin(), [&indices](int i) { return indices[i]; });
      std::sort(selected_points.begin(), selected_points.end());

    } else {
      auto kdtree = std::make_shared<gtsam_points::KdTree2<gtsam_points::PointCloud>>(submap->frame);
      auto rg = gtsam_points::region_growing_init(*submap->frame, *kdtree, picked_point.homogeneous(), region_growing_params);
      gtsam_points::region_growing_update(rg, *submap->frame, *kdtree, region_growing_params);

      selected_points.resize(rg.cluster_indices.size());
      neg_selected_points.resize(submap->frame->size() - rg.cluster_indices.size());
      std::copy(rg.cluster_indices.begin(), rg.cluster_indices.end(), selected_points.begin());
      std::sort(selected_points.begin(), selected_points.end());
    }

    int selected_cursor = 0;
    int neg_selected_cursor = 0;
    neg_selected_points.resize(submap->frame->size() - selected_points.size());
    for (size_t i = 0; i < submap->frame->size(); i++) {
      if (selected_cursor < selected_points.size() && selected_points[selected_cursor] == i) {
        selected_cursor++;
      } else {
        neg_selected_points[neg_selected_cursor++] = i;
      }
    }

    auto drawable = std::make_shared<glk::IndexedPointCloudBuffer>(submap_drawable, reinterpret_cast<unsigned int*>(selected_points.data()), selected_points.size());
    guik::viewer()->update_drawable("selected_points", drawable, guik::FlatOrange().set_point_scale(3.0));
  }

  void remove_points() {
    if (selected_points.empty() || neg_selected_points.empty()) {
      return;
    }

    submap->frame = gtsam_points::sample(submap->frame, neg_selected_points);
    submap_drawable = std::make_shared<glk::PointCloudBuffer>(submap->frame->points, submap->frame->size());

    selected_points.clear();
    neg_selected_points.clear();

    auto viewer = guik::viewer();
    viewer->remove_drawable("selected_points");
  }

private:
  std::shared_ptr<spdlog::logger> logger;
  std::unique_ptr<guik::ModelControl> model_control;

  glim::SubMap::Ptr submap;
  glk::PointCloudBuffer::Ptr submap_drawable;

  bool show_selection_radius;

  bool draw_gizmo;
  int selected_tool;

  float select_radius;

  bool show_segmentation_radius;
  int segmentation_method;
  gtsam_points::MinCutParams min_cut_params;
  gtsam_points::RegionGrowingParams region_growing_params;

  std::vector<int> selected_points;
  std::vector<int> neg_selected_points;

  Eigen::Vector3d picked_point;
};

////////////////////////////////////////////////////////////////////////
MapEditor::MapEditor(const std::string& init_map_path) : init_map_path(init_map_path) {
  logger = get_default_logger();

  selected_submap = 0;

  auto viewer = guik::viewer();
  viewer->sub_viewer("global")->use_orbit_camera_control(200.0, 0.0, -80.0 * M_PI / 180.0);

  viewer->register_ui_callback("log", guik::create_logger_ui(get_ringbuffer_sink(), 0.8));
  viewer->register_ui_callback("ui_callback", [this] { ui_callback(); });

  progress_modal.reset(new guik::ProgressModal("progress"));

  const int num_threads = std::thread::hardware_concurrency();
  logger->info("Using {} threads", num_threads);

  submap_editor.reset(new SubMapEditor(logger, num_threads));
}

MapEditor::~MapEditor() {}

void MapEditor::run() {
  guik::viewer()->spin();
}

void MapEditor::update_viewer() {
  auto viewer = guik::viewer();
  auto sub_viewer = viewer->sub_viewer("global");

  if (submaps.empty()) {
    viewer->clear_drawables();
    sub_viewer->clear_drawables();
    return;
  }

  if (submap_drawables.size() != submaps.size()) {
    submap_drawables.resize(submaps.size());
    for (size_t i = 0; i < submaps.size(); i++) {
      const auto& submap = submaps[i];
      submap_drawables[i] = std::make_shared<glk::PointCloudBuffer>(submap->frame->points, submap->frame->size());
      sub_viewer->update_drawable(fmt::format("submap_{}", i), submap_drawables[i], guik::Rainbow(submap->T_world_origin));
    }
  }

  if (selected_submap < submaps.size()) {
    sub_viewer->update_drawable("selected_submap", submap_drawables[selected_submap], guik::FlatOrange(submaps[selected_submap]->T_world_origin).set_point_scale(3.0));
  }
}

void MapEditor::main_menu() {
  bool start_open_map = false;
  bool start_save_map = false;

  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      if (ImGui::MenuItem("Open New Map")) {
        if (!submaps.empty()) {
          if (pfd::message("Warning", "Close the current map?").result() == pfd::button::ok) {
            submaps.clear();
            submap_drawables.clear();
            update_viewer();
            submap_editor->set_submap(nullptr);
            start_open_map = true;
          }
        } else {
          start_open_map = true;
        }
      }

      if (ImGui::MenuItem("Close map")) {
        if (submaps.empty()) {
          logger->warn("No map to close");
        } else if (pfd::message("Warning", "Close the map?").result() == pfd::button::ok) {
          submaps.clear();
          submap_drawables.clear();
          update_viewer();
          submap_editor->set_submap(nullptr);
        }
      }

      if (ImGui::MenuItem("Save map")) {
        if (submaps.empty()) {
          logger->warn("No map to save");
        } else {
          start_save_map = true;
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
      map_path = pfd::select_folder("Select a dump directory", recent_files.most_recent()).result();
    } else {
      map_path = init_map_path;
      init_map_path.clear();
    }

    if (!map_path.empty()) {
      logger->info("Load map from {}", map_path);
      recent_files.push(map_path);

      progress_modal->open<std::vector<glim::SubMap::Ptr>>("open", [this](guik::ProgressInterface& progress) { return load_submaps(progress, map_path); });
    }
  }

  const auto loaded_submaps = progress_modal->run<std::vector<glim::SubMap::Ptr>>("open");
  if (loaded_submaps && !loaded_submaps->empty()) {
    selected_submap = 0;
    submaps = *loaded_submaps;

    update_viewer();
    submap_editor->set_submap(submaps[selected_submap]);
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

  if (ImGui::Begin("submap editor", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    // Submap selection
    const int old_selected_submap = selected_submap;
    ImGui::DragInt("submap_id", &selected_submap, 1, 0, submaps.size() - 1);
    ImGui::SameLine();
    ImGui::PushButtonRepeat(true);
    selected_submap += ImGui::ArrowButton("Dec", ImGuiDir_Left) ? -1 : 0;
    ImGui::SameLine();
    selected_submap += ImGui::ArrowButton("Inc", ImGuiDir_Right) ? 1 : 0;
    ImGui::PopButtonRepeat();
    selected_submap = std::max<int>(0, std::min<int>(static_cast<int>(submaps.size()) - 1, selected_submap));

    if (old_selected_submap != selected_submap && selected_submap < submaps.size()) {
      update_viewer();
      submap_editor->set_submap(submaps[selected_submap]);
    }

    ImGui::Separator();
    submap_editor->draw_ui();

    ImGui::End();
  }
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
