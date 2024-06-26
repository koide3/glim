#include <glim/viewer/interactive/bundle_adjustment_modal.hpp>

#include <boost/format.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam_points/factors/bundle_adjustment_factor_evm.hpp>
#include <gtsam_points/factors/bundle_adjustment_factor_lsq.hpp>

#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/gl_canvas.hpp>
#include <guik/progress_modal.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace glim {

BundleAdjustmentModal::BundleAdjustmentModal() {
  request_to_open = false;

  radius = 1.0f;
  min_radius = 0.1f;
  max_radius = 5.0f;
  min_points = 5;
  max_points = 8192;
  plane_eps = 0.01f;

  num_points = 0;
  eigenvalues.setZero();

  center.setZero();
  canvas.reset(new guik::GLCanvas(Eigen::Vector2i(512, 512)));
  progress_modal.reset(new guik::ProgressModal("bundle_adjustment_progress"));
}

BundleAdjustmentModal::~BundleAdjustmentModal() {}

void BundleAdjustmentModal::set_frames(const std::vector<SubMap::ConstPtr>& submaps, const std::vector<Eigen::Isometry3d>& submap_poses, const Eigen::Vector3d& center) {
  //
  this->submaps.clear();
  this->submap_poses.clear();

  for (int i = 0; i < submaps.size(); i++) {
    if ((submap_poses[i].translation() - center).norm() > 25.0) {
      continue;
    }

    this->submaps.push_back(submaps[i]);
    this->submap_poses.push_back(Eigen::Translation3d(-center) * submap_poses[i]);
    this->submap_drawables.push_back(std::make_shared<glk::PointCloudBuffer>(submaps[i]->frame->points, submaps[i]->frame->size()));
  }

  this->radius = 1.0f;
  this->center << center, 1.0;

  const auto points = extract_points(radius);
  this->num_points = points.size();
  this->eigenvalues = calc_eigenvalues(points);

  request_to_open = true;
}

gtsam::NonlinearFactor::shared_ptr BundleAdjustmentModal::run() {
  if (request_to_open) {
    ImGui::OpenPopup("bundle adjustment");
    request_to_open = false;
  }

  gtsam::NonlinearFactor::shared_ptr factor;

  if (ImGui::BeginPopupModal("bundle adjustment", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    // Draw canvas
    ImGui::BeginChild(
      "canvas",
      ImVec2(512, 512),
      false,
      ImGuiWindowFlags_ChildWindow | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_NoNavFocus);
    if (ImGui::IsWindowFocused()) {
      canvas->mouse_control();
    }
    draw_canvas();
    ImGui::Image((void*)canvas->frame_buffer->color().id(), ImVec2(512, 512), ImVec2(0, 1), ImVec2(1, 0));
    ImGui::EndChild();

    if (submaps.size() < 2) {
      ImGui::Text("The number of selected submaps is smaller than 2!!");
      ImGui::Text("Close this modal an reselect a point!!");
    }

    ImGui::DragFloat("Radius", &radius, 0.01f, min_radius, max_radius);

    if (ImGui::Button("Update")) {
      update_indicator();
    }
    ImGui::SameLine();
    ImGui::Text("Points:%d Eigenvalues:%.3f %.3f %.3f", num_points, eigenvalues[0], eigenvalues[1], eigenvalues[2]);

    ImGui::Separator();
    ImGui::DragFloatRange2("Radius Range", &min_radius, &max_radius, 0.01f, 0.01f, 100.0f);
    ImGui::DragIntRange2("Num points Range", &min_points, &max_points, 1, 2, 8192 * 10);
    ImGui::DragFloat("Plane eps", &plane_eps, 0.0001f, 0.0001f, 0.1f);

    if (ImGui::Button("Auto Radius")) {
      progress_modal->open<double>("auto radius", [this](guik::ProgressInterface& progress) { return auto_radius(progress); });
    }
    auto auto_radius_result = progress_modal->run<double>("auto radius");
    if (auto_radius_result) {
      radius = *auto_radius_result;
      update_indicator();
    }

    ImGui::Separator();

    if (ImGui::Button("Create Factor")) {
      factor = create_factor();

      submaps.clear();
      submap_poses.clear();
      submap_drawables.clear();
      ImGui::CloseCurrentPopup();
    }

    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
      submaps.clear();
      submap_poses.clear();
      submap_drawables.clear();
      ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
  }

  return factor;
}

void BundleAdjustmentModal::update_indicator() {
  const auto points = extract_points(radius);
  num_points = points.size();
  eigenvalues = calc_eigenvalues(points);
}

std::vector<std::pair<int, int>> BundleAdjustmentModal::extract_points(double radius) {
  const double radius_sq = radius * radius;
  std::vector<std::pair<int, int>> points_in_radius;

  for (int i = 0; i < submaps.size(); i++) {
    const int submap_id = submaps[i]->id;
    const auto& frame = submaps[i]->frame;

    for (int j = 0; j < frame->size(); j++) {
      Eigen::Vector4d pt = submap_poses[i] * frame->points[j];
      pt.w() = 0.0;
      if (pt.squaredNorm() < radius_sq) {
        points_in_radius.push_back(std::make_pair(i, j));
      }
    }
  }

  return points_in_radius;
}

Eigen::Vector3d BundleAdjustmentModal::calc_eigenvalues(const std::vector<std::pair<int, int>>& point_indices) {
  Eigen::Vector4d sum_pts = Eigen::Vector4d::Zero();
  Eigen::Matrix4d sum_cross = Eigen::Matrix4d::Zero();

  for (const auto& submap_point : point_indices) {
    const int submap_index = submap_point.first;
    const int point_index = submap_point.second;

    const Eigen::Vector4d pt = submap_poses[submap_index] * submaps[submap_index]->frame->points[point_index];

    sum_pts += pt;
    sum_cross += pt * pt.transpose();
  }

  Eigen::Vector4d mean = sum_pts / point_indices.size();
  Eigen::Matrix4d cov = (sum_cross - mean * sum_pts.transpose()) / point_indices.size();

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig;
  eig.computeDirect(cov.block<3, 3>(0, 0));

  return eig.eigenvalues();
}

double BundleAdjustmentModal::auto_radius(guik::ProgressInterface& progress) {
  progress.set_title("Auto radius estimation");
  progress.set_text("Extracting initial points");
  progress.set_maximum(10);
  double current_radius = radius;
  auto extracted_points = extract_points(current_radius);
  Eigen::Vector3d eigenvalues = calc_eigenvalues(extracted_points);

  for (int i = 0; i < 10; i++) {
    progress.set_text((boost::format("Trial %d/%d R:%.3f") % i % 10 % current_radius).str());
    progress.increment();

    double trial_radius;
    if (eigenvalues[0] / eigenvalues[2] > plane_eps) {
      trial_radius = current_radius * 0.8;
    } else {
      trial_radius = current_radius * 1.1;
    }

    if (trial_radius < min_radius || trial_radius > max_radius) {
      break;
    }

    auto points = extract_points(trial_radius);

    if (points.size() < 10) {
      break;
    }

    Eigen::Vector3d values = calc_eigenvalues(points);

    if (trial_radius > radius && values[0] / values[2] > plane_eps) {
      break;
    }

    extracted_points = points;
    eigenvalues = values;
    current_radius = trial_radius;
  }

  return current_radius;
}

gtsam::NonlinearFactor::shared_ptr BundleAdjustmentModal::create_factor() {
  using gtsam::symbol_shorthand::X;

  const auto extracted_points = extract_points(radius);

  gtsam_points::PlaneEVMFactor::shared_ptr factor(new gtsam_points::PlaneEVMFactor());
  for (const auto& submap_point : extracted_points) {
    const int submap_index = submap_point.first;
    const int point_index = submap_point.second;

    const auto& point = submaps[submap_index]->frame->points[point_index];

    factor->add(point.head<3>(), X(submaps[submap_index]->id));
  }

  return factor;
}

void BundleAdjustmentModal::draw_canvas() {
  canvas->bind();
  canvas->shader->set_uniform("point_scale", 0.1f);
  canvas->shader->set_uniform("color_mode", guik::ColorMode::RAINBOW);
  canvas->shader->set_uniform("material_color", Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));

  for (int i = 0; i < submaps.size(); i++) {
    canvas->shader->set_uniform("model_matrix", submap_poses[i].cast<float>().matrix());
    submap_drawables[i]->draw(*canvas->shader);
  }

  canvas->unbind();

  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);

  canvas->bind_second();

  canvas->shader->set_uniform("color_mode", guik::ColorMode::FLAT_COLOR);
  canvas->shader->set_uniform("material_color", Eigen::Vector4f(1.0f, 0.0f, 0.0f, 0.5f));
  canvas->shader->set_uniform("model_matrix", (Eigen::Isometry3f::Identity() * Eigen::UniformScaling(radius)).matrix());

  glk::Primitives::sphere()->draw(*canvas->shader);

  canvas->unbind_second();

  glDisable(GL_CULL_FACE);
}

}  // namespace glim