#include <glim/viewer/editor/points_selector.hpp>

#include <glk/drawable_container.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <gtsam_points/util/vector3i_hash.hpp>
#include <gtsam_points/util/fast_floor.hpp>

namespace glim {

PointsSelector::PointsSelector(std::shared_ptr<spdlog::logger> logger)
: map_cell_resolution(2.0f),
  cell_selection_window(5),
  submaps(submaps),
  picked_point(0, 0, 0),
  logger(logger) {}

PointsSelector::~PointsSelector() {}

void PointsSelector::clear() {
  this->submaps.clear();
  this->map_cells.clear();
  this->selected_cells.clear();
  this->picked_point.setZero();
}

void PointsSelector::set_submaps(const std::vector<glim::SubMap::Ptr>& submaps) {
  this->submaps = submaps;
  update_cells();
}

void PointsSelector::update_cells() {
  map_cells.clear();
  selected_cells.clear();

  logger->info("Update cells");
  const double inv_resolution = 1.0 / map_cell_resolution;
  for (const auto& submap : submaps) {
    for (int i = 0; i < submap->frame->size(); i++) {
      const Eigen::Vector4d pt = submap->T_world_origin * submap->frame->points[i];
      const Eigen::Vector3i coord = gtsam_points::fast_floor(pt * inv_resolution).head<3>();

      auto& cell = map_cells[coord];
      if (!cell) {
        cell = std::make_shared<MapCell>(map_cell_resolution, coord);
      }
      cell->add_point(submap, i);
    }
  }

  auto viewer = guik::viewer();
  viewer->remove_drawable(std::regex("cell_.*"));
  for (const auto& cell : map_cells) {
    cell.second->finalize();
    viewer->update_drawable(cell.second->name(), cell.second->cloud_buffer, guik::Rainbow());
  }

  logger->info("|cells|={}", map_cells.size());
}

// void PointsSelector::select(const Eigen::Vector3d& point, const std::vector<MapCell::Ptr>& selected_cells) {
//   this->picked_point = point;
//   this->selected_cells = selected_cells;

//   auto container = std::make_shared<glk::DrawableContainer>();
//   for (const auto& cell : selected_cells) {
//     container->push_back(cell->cloud_buffer);
//   }

//   auto viewer = guik::viewer();
//   viewer->update_drawable("selected_cells", container, guik::FlatOrange().add("point_scale", 2.0f));
// }

void PointsSelector::draw_ui() {
  auto viewer = guik::viewer();

  if (ImGui::Begin("editor", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("Submaps: %ld", submaps.size());
    ImGui::Separator();
    ImGui::DragFloat("map cell resolution", &map_cell_resolution, 0.1f, 0.1f, 10.0f);
    ImGui::DragInt("selection window", &cell_selection_window, 1, 1, 10);
    if (ImGui::Button("Update cells")) {
      update_cells();
    }

    ImGui::End();
  }

  auto picked_point = viewer->pick_point(1, 3);
  if (picked_point) {
    const Eigen::Vector3i center = (picked_point->cast<double>().array() / map_cell_resolution).floor().cast<int>();

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

    auto container = std::make_shared<glk::DrawableContainer>();
    for (const auto& cell : selected_cells) {
      container->push_back(cell->cloud_buffer);
    }
    viewer->update_drawable("selected", container, guik::FlatOrange().add("point_scale", 2.0f));

    logger->info("|selected_cells|={}", selected_cells.size());
    this->selected_cells = std::move(selected_cells);
    this->picked_point = picked_point->cast<double>();

    ImGui::OpenPopup("context_menu");
  }

  if (ImGui::BeginPopup("context_menu", ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("Point: (%.1f, %.1f, %.1f)", this->picked_point.x(), this->picked_point.y(), this->picked_point.z());
    ImGui::Text("Selected cells: %ld", selected_cells.size());
    ImGui::Separator();

    ImGui::EndPopup();
  }
}
}  // namespace glim