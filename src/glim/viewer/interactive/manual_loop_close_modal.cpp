#include <glim/viewer/interactive/manual_loop_close_modal.hpp>

#include <boost/format.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_points/types/point_cloud.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>

#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/gl_canvas.hpp>
#include <guik/model_control.hpp>
#include <guik/progress_modal.hpp>
#include <guik/viewer/light_viewer.hpp>

#ifdef GTSAM_USE_TBB
#include <tbb/task_arena.h>
#endif

namespace glim {

ManualLoopCloseModal::ManualLoopCloseModal() : request_to_open(false) {
  target_pose.setIdentity();
  source_pose.setIdentity();

  information_scale = 1.0f;
  max_correspondence_distance = 1.0f;

  canvas.reset(new guik::GLCanvas(Eigen::Vector2i(512, 512)));
  progress_modal.reset(new guik::ProgressModal("manual_loop_close_progress"));
  model_control.reset(new guik::ModelControl("model_control"));

#ifdef GTSAM_USE_TBB
  tbb_task_arena = std::make_shared<tbb::task_arena>(1);
#endif
}

ManualLoopCloseModal::~ManualLoopCloseModal() {}

void ManualLoopCloseModal::set_target(const gtsam::Key target_key, const gtsam_points::PointCloud::ConstPtr& target, const Eigen::Isometry3d& target_pose) {
  this->target_key = target_key;
  this->target = target;
  this->target_pose = target_pose;
  this->target_drawable = std::make_shared<glk::PointCloudBuffer>(target->points, target->size());
}

void ManualLoopCloseModal::set_source(const gtsam::Key source_key, const gtsam_points::PointCloud::ConstPtr& source, const Eigen::Isometry3d& source_pose) {
  this->source_key = source_key;
  this->source = source;
  this->source_pose = source_pose;
  this->source_drawable = std::make_shared<glk::PointCloudBuffer>(source->points, source->size());
  request_to_open = true;
}

gtsam::NonlinearFactor::shared_ptr ManualLoopCloseModal::run() {
  if (request_to_open && target && source) {
    ImGui::OpenPopup("manual loop close");
    model_control->set_model_matrix((target_pose.inverse() * source_pose).cast<float>().matrix());
  }
  request_to_open = false;

  gtsam::NonlinearFactor::shared_ptr factor;

  if (ImGui::BeginPopupModal("manual loop close", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    // Draw canvas
    ImGui::BeginChild(
      "canvas",
      ImVec2(512, 512),
      false,
      ImGuiWindowFlags_ChildWindow | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_NoNavFocus);
    if (ImGui::IsWindowFocused() && !model_control->is_guizmo_using()) {
      canvas->mouse_control();
    }
    draw_canvas();
    ImGui::Image((void*)canvas->frame_buffer->color().id(), ImVec2(512, 512), ImVec2(0, 1), ImVec2(1, 0));

    ImVec2 canvas_rect_min = ImGui::GetItemRectMin();
    ImVec2 canvas_rect_max = ImGui::GetItemRectMax();

    model_control->draw_gizmo(
      canvas_rect_min.x,
      canvas_rect_min.y,
      canvas_rect_max.x - canvas_rect_min.x,
      canvas_rect_max.y - canvas_rect_min.y,
      canvas->camera_control->view_matrix(),
      canvas->projection_control->projection_matrix(),
      true);

    ImGui::EndChild();

    model_control->draw_gizmo_ui();

    ImGui::DragFloat("max_corr_dist", &max_correspondence_distance, 0.01f, 0.01f, 100.0f);
    ImGui::DragFloat("inf_scale", &information_scale, 0.0f, 1.0f, 10000.0f);

    bool open_modal = false;
    if (ImGui::Button("Align")) {
      open_modal = true;
    }

    if (open_modal) {
      progress_modal->open<std::shared_ptr<Eigen::Isometry3d>>("align", [this](guik::ProgressInterface& progress) { return align(progress); });
    }
    auto align_result = progress_modal->run<std::shared_ptr<Eigen::Isometry3d>>("align");
    if (align_result) {
      model_control->set_model_matrix((*align_result)->cast<float>().matrix());
    }

    if (ImGui::Button("Create Factor")) {
      factor = create_factor();
      ImGui::CloseCurrentPopup();
      target = nullptr;
      source = nullptr;
    }

    if (ImGui::Button("Cancel")) {
      ImGui::CloseCurrentPopup();
      target = nullptr;
      source = nullptr;
    }

    ImGui::EndPopup();
  }

  return factor;
}

std::shared_ptr<Eigen::Isometry3d> ManualLoopCloseModal::align(guik::ProgressInterface& progress) {
  progress.set_title("Aligning frames");
  progress.set_maximum(20);

  progress.set_text("Creating graph");
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(0, gtsam::Pose3::Identity(), gtsam::noiseModel::Isotropic::Precision(6, 1e6));

  auto factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(0, 1, target, source);
  factor->set_num_threads(4);
  factor->set_max_correspondence_distance(max_correspondence_distance);
  graph.add(factor);

  gtsam::Values values;
  values.insert(0, gtsam::Pose3::Identity());
  values.insert(1, gtsam::Pose3(model_control->model_matrix().cast<double>()));

  progress.set_text("Optimizing");
  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setMaxIterations(20);
  lm_params.callback = [&](const gtsam_points::LevenbergMarquardtOptimizationStatus& status, const gtsam::Values& values) {
    progress.increment();
    progress.set_text((boost::format("Optimizing iter:%d error:%.3f") % status.iterations % status.error).str());
  };

  gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);

#ifdef GTSAM_USE_TBB
  auto arena = static_cast<tbb::task_arena*>(tbb_task_arena.get());
  arena->execute([&] {
#endif
    values = optimizer.optimize();
#ifdef GTSAM_USE_TBB
  });
#endif

  const gtsam::Pose3 estimated = values.at<gtsam::Pose3>(0).inverse() * values.at<gtsam::Pose3>(1);

  return std::shared_ptr<Eigen::Isometry3d>(new Eigen::Isometry3d(estimated.matrix()));
}

gtsam::NonlinearFactor::shared_ptr ManualLoopCloseModal::create_factor() {
  const gtsam::Pose3 relative(model_control->model_matrix().cast<double>());

  gtsam::Values values;
  values.insert(0, gtsam::Pose3::Identity());
  values.insert(1, relative);

  auto factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(0, 1, target, source);
  factor->set_num_threads(4);
  factor->set_max_correspondence_distance(max_correspondence_distance);

  const auto linearized = factor->linearize(values);
  const auto H = linearized->hessianBlockDiagonal()[1];

  return gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(target_key, source_key, relative, gtsam::noiseModel::Gaussian::Information(information_scale * H));
}

void ManualLoopCloseModal::draw_canvas() {
  canvas->bind();
  canvas->shader->set_uniform("color_mode", guik::ColorMode::FLAT_COLOR);
  canvas->shader->set_uniform("material_color", Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
  canvas->shader->set_uniform("model_matrix", Eigen::Matrix4f::Identity().eval());

  target_drawable->draw(*canvas->shader);

  canvas->shader->set_uniform("material_color", Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f));
  canvas->shader->set_uniform("model_matrix", model_control->model_matrix());

  source_drawable->draw(*canvas->shader);

  canvas->unbind();
}

}  // namespace glim