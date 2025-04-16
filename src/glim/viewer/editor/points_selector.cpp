#include <glim/viewer/editor/points_selector.hpp>

#include <glk/html_colors.hpp>
#include <glk/drawable_container.hpp>
#include <guik/model_control.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <gtsam_points/util/vector3i_hash.hpp>
#include <gtsam_points/util/fast_floor.hpp>
#include <gtsam_points/util/normal_estimation.hpp>
#include <gtsam_points/ann/kdtree2.hpp>

namespace glim {

PointsSelector::PointsSelector(std::shared_ptr<spdlog::logger> logger) : picked_point(0, 0, 0), logger(logger) {
  model_control.reset(new guik::ModelControl("model_control"));
  model_control->set_gizmo_operation("UNIVERSAL");

  map_cell_resolution = 2.0f;
  cell_selection_window = 5;

  show_picked_point = false;
  show_selection_radius = false;

  show_gizmo = false;
  selected_tool = 0;
  select_radius = 0.5f;

  show_segmentation_radius = false;
  segmentation_method = 0;

  min_cut_params.foreground_weight = 10.0;
}

PointsSelector::~PointsSelector() {}

void PointsSelector::clear() {
  this->submaps.clear();
  this->map_cells.clear();
  this->picked_point.setZero();
}

void PointsSelector::set_submaps(const std::vector<glim::SubMap::Ptr>& submaps) {
  this->submaps = submaps;
  this->submap_drawables.resize(submaps.size());
  std::transform(submaps.begin(), submaps.end(), this->submap_drawables.begin(), [](const SubMap::Ptr& submap) {
    return guik::viewer()->update_points("submap_" + std::to_string(submap->id), submap->frame->points, submap->frame->size(), guik::Rainbow(submap->T_world_origin));
  });

  update_cells();
}

gtsam_points::PointCloudCPU::Ptr PointsSelector::collect_submap_points(const std::vector<std::uint64_t>& point_ids) {
  auto points = std::make_shared<gtsam_points::PointCloudCPU>();
  points->points_storage.reserve(point_ids.size());
  points->normals_storage.reserve(point_ids.size());
  points->covs_storage.reserve(point_ids.size());
  points->intensities_storage.reserve(point_ids.size());

  for (std::uint64_t x : point_ids) {
    const std::uint64_t submap_id = x >> 32;
    const std::uint64_t point_id = x & ((1ull << 32) - 1);

    const auto submap = submaps[submap_id];

    points->points_storage.emplace_back(submap->T_world_origin * submap->frame->points[point_id]);
    if (submaps[submap_id]->frame->has_normals()) {
      points->normals_storage.emplace_back(submap->T_world_origin.matrix() * submap->frame->normals[point_id]);
    }
    if (submaps[submap_id]->frame->has_covs()) {
      points->covs_storage.emplace_back(submap->T_world_origin.matrix() * submap->frame->covs[point_id] * submap->T_world_origin.matrix().transpose());
    }
    if (submaps[submap_id]->frame->has_intensities()) {
      points->intensities_storage.emplace_back(submaps[submap_id]->frame->intensities[point_id]);
    }
  }

  points->num_points = points->points_storage.size();
  points->points = points->points_storage.data();
  points->normals = points->normals_storage.data();
  points->covs = points->covs_storage.data();
  points->intensities = points->intensities_storage.data();

  return points;
}

void PointsSelector::update_cells() {
  cell_map.clear();
  map_cells.clear();

  logger->info("Update cells");
  const double inv_resolution = 1.0 / map_cell_resolution;
  for (const auto& submap : submaps) {
    auto& map = cell_map[submap];

    for (int i = 0; i < submap->frame->size(); i++) {
      const Eigen::Vector4d pt = submap->T_world_origin * submap->frame->points[i];
      const Eigen::Vector3i coord = gtsam_points::fast_floor(pt * inv_resolution).head<3>();

      auto& cell = map_cells[coord];
      if (!cell) {
        cell = std::make_shared<MapCell>(map_cell_resolution, coord);
      }
      cell->add_point(submap, i);
      map.insert(cell);
    }
  }

  logger->info("|cells|={}", map_cells.size());
}

void PointsSelector::draw_ui() {
  auto viewer = guik::viewer();

  const auto show_note = [](const std::string& text) {
    if (ImGui::IsItemHovered()) {
      ImGui::BeginTooltip();
      ImGui::Text("%s", text.c_str());
      ImGui::EndTooltip();
    }
    return false;
  };

  if (ImGui::Begin("editor", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("Submaps: %ld", submaps.size());
    ImGui::Separator();
    ImGui::DragFloat("map cell resolution", &map_cell_resolution, 0.1f, 0.1f, 10.0f);
    ImGui::DragInt("selection window", &cell_selection_window, 1, 1, 10);
    if (ImGui::Button("Update cells")) {
      update_cells();
    }

    // Gizmo selection tool
    ImGui::Separator();
    ImGui::Checkbox("##Show_gizmo", &show_gizmo);
    show_note("Show gizmo for selection tool");

    ImGui::SameLine();
    ImGui::SetNextItemWidth(150);
    ImGui::Combo("Selection tool", &selected_tool, "Box\0Sphere\0");

    if (ImGui::Button("Select points") || show_note("Select points within the selection tool")) {
      select_points_tool();
    }

    ImGui::SameLine();
    if (ImGui::Button("Reset gizmo")) {
      model_control->set_model_matrix(Eigen::Matrix4d::Identity().eval());
    }

    if (show_gizmo) {
      model_control->draw_gizmo();
    }

    ImGui::Separator();
    if (ImGui::Button("Unselect points") || show_note("Unselect all points")) {
      selected_point_ids.clear();
      viewer->remove_drawable("selected");
    }

    if (ImGui::Button("Remove selected points") || (ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_R)) || show_note("Remove selected points from the submap")) {
      remove_points();
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(Ctrl+R)");

    ImGui::End();
  }

  // Clear selection by left clicking
  if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !ImGui::IsAnyItemHovered()) {
    // selected_point_ids.clear();
    // viewer->remove_drawable("selected");
  }

  // Select cells based on the picked point position
  auto pick = viewer->pick_point(1, 3);
  if (pick) {
    const Eigen::Vector3i center = (pick->cast<double>().array() / map_cell_resolution).floor().cast<int>();

    std::vector<MapCell::Ptr> selected_cells;
    const int w = cell_selection_window;
    for (int i = -w; i <= w; i++) {
      for (int j = -w; j <= w; j++) {
        for (int k = -w; k <= w; k++) {
          const Eigen::Vector3i coord = center + Eigen::Vector3i(i, j, k);
          auto it = map_cells.find(coord);
          if (it != map_cells.end()) {
            selected_cells.push_back(it->second);
          }
        }
      }
    }

    std::vector<std::uint64_t> selected_point_ids;
    for (const auto& cell : selected_cells) {
      selected_point_ids.insert(selected_point_ids.end(), cell->point_ids.begin(), cell->point_ids.end());
    }

    logger->info("|selected_cells|={} |selected_points|={}", selected_cells.size(), selected_point_ids.size());

    this->selected_point_ids = std::move(selected_point_ids);
    this->selected_points = collect_submap_points(this->selected_point_ids);
    this->picked_point = pick->cast<double>();

    viewer->update_points("selected", this->selected_points->points, this->selected_points->size(), guik::FlatOrange().set_alpha(0.8).set_point_scale(2.0f));

    ImGui::OpenPopup("context_menu");
  }

  show_picked_point = false;
  show_selection_radius = false;
  show_segmentation_radius = false;

  // Context menu
  if (ImGui::BeginPopup("context_menu", ImGuiWindowFlags_AlwaysAutoResize)) {
    show_picked_point = true;

    ImGui::Text("Point: (%.1f, %.1f, %.1f)", this->picked_point.x(), this->picked_point.y(), this->picked_point.z());
    ImGui::Text("Selected points: %ld", selected_point_ids.size());
    ImGui::Separator();

    // Look at
    if (ImGui::MenuItem("Move camera here")) {
      viewer->lookat(picked_point);
      viewer->reset_center();
    }

    // Gizmo
    const bool gizmo_menu_open = ImGui::BeginMenu("Selection tool (Gizmo)");
    const bool gizmo_selection_clicked = ImGui::IsItemClicked(ImGuiMouseButton_Left);

    if (gizmo_menu_open) {
      if (ImGui::MenuItem("Show gizmo here") || gizmo_selection_clicked) {
        Eigen::Matrix4f matrix = model_control->model_matrix();
        matrix.block<3, 1>(0, 3) = this->picked_point.cast<float>();
        model_control->set_model_matrix(matrix);
        show_gizmo = true;
        selected_point_ids.clear();
      }

      if (ImGui::MenuItem("Hide gizmo")) {
        show_gizmo = false;
        selected_point_ids.clear();
      }

      if (ImGui::MenuItem("Select")) {
        logger->info("Select points with gizmo clicked");
        select_points_tool();
      }

      ImGui::EndMenu();
    }

    if (gizmo_selection_clicked) {
      ImGui::CloseCurrentPopup();
    }

    // Radius
    const bool radius_menu_open = ImGui::BeginMenu("Select in radius");
    bool radius_selection_clicked = ImGui::IsItemClicked(ImGuiMouseButton_Left);

    if (radius_menu_open) {
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

    // Segmentation
    const bool segmentation_menu_open = ImGui::BeginMenu("Segmentation");
    bool segmentation_clicked = ImGui::IsItemClicked(ImGuiMouseButton_Left);

    if (segmentation_menu_open) {
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

      segmentation_clicked |= ImGui::MenuItem("Segment");

      ImGui::EndMenu();
    }

    if (segmentation_clicked) {
      logger->info("Segmentation clicked");
      select_points_segmentation();
      ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
  }

  if (selected_point_ids.empty()) {
    viewer->remove_drawable("selected");
  }

  if (show_picked_point) {
    viewer->update_sphere("picked_point", guik::FlatRed().translate(this->picked_point).scale(0.2));
  } else {
    viewer->remove_drawable("picked_point");
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

  if (show_gizmo) {
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

void PointsSelector::remove_points() {
  std::sort(selected_point_ids.begin(), selected_point_ids.end());

  std::vector<SubMap::Ptr> updated_submaps;
  std::unordered_set<MapCell::Ptr> cells_to_update;

  auto point_id_itr = selected_point_ids.begin();
  while (point_id_itr != selected_point_ids.end()) {
    const std::uint64_t submap_id = *point_id_itr >> 32;

    const auto begin = point_id_itr;
    const auto end = std::find_if(point_id_itr, selected_point_ids.end(), [=](std::uint64_t id) {
      const int submap_id_ = id >> 32;
      return submap_id_ != submap_id;
    });
    point_id_itr = end;

    // find points to be removed in the submap
    auto submap = submaps[submap_id];
    updated_submaps.emplace_back(submap);

    std::vector<int> point_indices(submap->frame->size());
    std::iota(point_indices.begin(), point_indices.end(), 0);

    std::for_each(begin, end, [&](std::uint64_t id) {
      const int point_id = id & ((1UL << 32) - 1);
      if (point_id > submap->frame->size()) {
        logger->warn("Invalid point id {} in submap {}", point_id, submap_id);
        return;
      }

      point_indices[point_id] = -1;
    });

    // remove points from the submap
    const auto remove_loc = std::remove_if(point_indices.begin(), point_indices.end(), [](int id) { return id == -1; });
    point_indices.erase(remove_loc, point_indices.end());

    submap->frame = gtsam_points::sample(submap->frame, point_indices);

    for (auto& cell : cell_map[submap]) {
      cell->remove_submap(submap_id);
    }
  }

  // Update cells
  const double inv_resolution = 1.0 / map_cell_resolution;
  for (const auto& submap : updated_submaps) {
    submap_drawables[submap->id] =
      guik::viewer()->update_points("submap_" + std::to_string(submap->id), submap->frame->points, submap->frame->size(), guik::Rainbow(submap->T_world_origin));

    for (int i = 0; i < submap->frame->size(); i++) {
      const Eigen::Vector4d pt = submap->T_world_origin * submap->frame->points[i];
      const Eigen::Vector3i coord = gtsam_points::fast_floor(pt * inv_resolution).head<3>();
      map_cells[coord]->add_point(submap, i);
    }
  }

  selected_point_ids.clear();
  selected_points.reset();
}

void PointsSelector::select_points_tool() {
  const Eigen::Matrix4d inv_model_matrix = model_control->model_matrix().cast<double>().inverse();

  selected_point_ids.clear();
  for (const auto& submap : submaps) {
    for (int i = 0; i < submap->frame->size(); i++) {
      const Eigen::Vector4d pt_ = submap->T_world_origin * submap->frame->points[i];
      const Eigen::Vector3d pt = (inv_model_matrix * pt_).head<3>();
      if (selected_tool == 0) {
        if ((pt.array() > Eigen::Array3d::Constant(-0.5)).all() && (pt.array() < Eigen::Array3d::Constant(0.5)).all()) {
          selected_point_ids.emplace_back((static_cast<std::uint64_t>(submap->id) << 32) | i);
        }
      } else {
        if (pt.norm() < 1.0) {
          selected_point_ids.emplace_back((static_cast<std::uint64_t>(submap->id) << 32) | i);
        }
      }
    }
  }

  if (selected_point_ids.empty()) {
    logger->warn("No points selected");
    return;
  }

  selected_points = collect_submap_points(selected_point_ids);
  guik::viewer()->update_points("selected", this->selected_points->points, this->selected_points->size(), guik::FlatOrange().set_alpha(0.8).set_point_scale(2.0f));
  show_gizmo = false;
}

void PointsSelector::select_points_radius() {
  if (selected_point_ids.empty()) {
    logger->warn("No points selected");
    return;
  }

  std::vector<std::uint64_t> new_selected_point_ids;
  std::vector<int> selected_indices;

  const double sq_thresh = select_radius * select_radius;
  for (int i = 0; i < selected_points->size(); i++) {
    const double dist_sq = (selected_points->points[i] - picked_point.homogeneous()).squaredNorm();
    if (dist_sq < sq_thresh) {
      selected_indices.emplace_back(i);
      new_selected_point_ids.emplace_back(selected_point_ids[i]);
    }
  }

  selected_points = gtsam_points::sample(selected_points, selected_indices);
  selected_point_ids = std::move(new_selected_point_ids);
  logger->info("Select points in radius: {} points selected", selected_points->size());

  guik::viewer()->update_points("selected", this->selected_points->points, this->selected_points->size(), guik::FlatOrange().set_alpha(0.8).set_point_scale(2.0f));
}

void PointsSelector::select_points_segmentation() {
  if (selected_point_ids.empty()) {
    logger->warn("No points selected");
    return;
  }

  std::vector<std::uint64_t> new_selected_point_ids;

  if (segmentation_method == 0) {
    std::vector<std::uint64_t> filtered_ids;
    const double sq_dist_thresh = std::pow(min_cut_params.background_mask_radius + 1.0, 2);
    for (int i = 0; i < selected_point_ids.size(); i++) {
      const double sq_dist = (selected_points->points[i] - picked_point.homogeneous()).squaredNorm();
      if (sq_dist < sq_dist_thresh) {
        filtered_ids.emplace_back(selected_point_ids[i]);
      }
    }

    auto filtered = collect_submap_points(filtered_ids);
    filtered->add_normals(gtsam_points::estimate_normals(filtered->points, filtered->size(), 20, 4));

    auto filtered_tree = std::make_shared<gtsam_points::KdTree2<gtsam_points::PointCloud>>(filtered);
    auto result = gtsam_points::min_cut(*filtered, *filtered_tree, picked_point.homogeneous(), min_cut_params);

    for (int i = 0; i < result.cluster_indices.size(); i++) {
      new_selected_point_ids.emplace_back(filtered_ids[result.cluster_indices[i]]);
    }

  } else {
    auto filtered = collect_submap_points(selected_point_ids);
    auto kdtree = std::make_shared<gtsam_points::KdTree2<gtsam_points::PointCloud>>(filtered);
    auto rg = gtsam_points::region_growing_init(*filtered, *kdtree, picked_point.homogeneous(), region_growing_params);
    gtsam_points::region_growing_update(rg, *filtered, *kdtree, region_growing_params);

    for (int i = 0; i < rg.cluster_indices.size(); i++) {
      new_selected_point_ids.emplace_back(selected_point_ids[rg.cluster_indices[i]]);
    }
  }

  selected_points = collect_submap_points(new_selected_point_ids);
  selected_point_ids = std::move(new_selected_point_ids);
  guik::viewer()->update_points("selected", this->selected_points->points, this->selected_points->size(), guik::FlatOrange().set_alpha(0.8).set_point_scale(2.0f));
  logger->info("Select points by segmentation: {} points selected", selected_points->size());
}

}  // namespace glim