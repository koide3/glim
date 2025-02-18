#include <glim/viewer/interactive/manual_loop_close_modal.hpp>

#include <spdlog/spdlog.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/ann/kdtree2.hpp>
#include <gtsam_points/ann/kdtreex.hpp>
#include <gtsam_points/types/point_cloud.hpp>
#include <gtsam_points/features/normal_estimation.hpp>
#include <gtsam_points/features/covariance_estimation.hpp>
#include <gtsam_points/features/fpfh_estimation.hpp>
#include <gtsam_points/factors/integrated_icp_factor.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/registration/graduated_non_convexity.hpp>
#include <gtsam_points/registration/impl/graduated_non_convexity_impl.hpp>
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

ManualLoopCloseModal::ManualLoopCloseModal(const std::shared_ptr<spdlog::logger>& logger, int num_threads) : num_threads(num_threads), request_to_open(false), logger(logger) {
  target_pose.setIdentity();
  source_pose.setIdentity();

  min_distance = 0.5f;
  fpfh_radius = 5.0f;

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
  this->target = gtsam_points::PointCloudCPU::clone(*target);
  this->target_pose = target_pose;
  this->target_drawable = std::make_shared<glk::PointCloudBuffer>(target->points, target->size());
}

void ManualLoopCloseModal::set_source(const gtsam::Key source_key, const gtsam_points::PointCloud::ConstPtr& source, const Eigen::Isometry3d& source_pose) {
  this->source_key = source_key;
  this->source = gtsam_points::PointCloudCPU::clone(*source);
  this->source_pose = source_pose;
  this->source_drawable = std::make_shared<glk::PointCloudBuffer>(source->points, source->size());
  request_to_open = true;
}

void ManualLoopCloseModal::set_submaps(const std::vector<SubMap::ConstPtr>& target_submaps, const std::vector<SubMap::ConstPtr>& source_submaps) {
  logger->info("|targets|={} |sources|={}", target_submaps.size(), source_submaps.size());

  this->target_key = -1;
  this->target = nullptr;
  this->target_pose.setIdentity();
  this->target_drawable = nullptr;
  this->target_submaps = target_submaps;

  this->source_key = -1;
  this->source = nullptr;
  this->source_pose.setIdentity();
  this->source_drawable = nullptr;
  this->source_submaps = source_submaps;

  request_to_open = true;
}

void ManualLoopCloseModal::clear() {
  target_key = -1;
  source_key = -1;
  target = nullptr;
  source = nullptr;
  target_drawable = nullptr;
  source_drawable = nullptr;
  target_submaps.clear();
  source_submaps.clear();
}

gtsam::NonlinearFactor::shared_ptr ManualLoopCloseModal::run() {
  gtsam::NonlinearFactor::shared_ptr factor;

  if (request_to_open && target && source) {
    ImGui::OpenPopup("manual loop close");
    model_control->set_model_matrix((target_pose.inverse() * source_pose).cast<float>().matrix());
  } else if (request_to_open && target_submaps.size() && source_submaps.size()) {
    ImGui::OpenPopup("preprocess maps");
    model_control->set_model_matrix(Eigen::Matrix4f::Identity().eval());
  }
  request_to_open = false;

  bool open_preprocess_modal = false;
  if (ImGui::BeginPopupModal("preprocess maps", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::DragFloat("Min distance", &min_distance, 0.01f, 0.01f, 100.0f);

    if (ImGui::Button("OK")) {
      open_preprocess_modal = true;
      ImGui::CloseCurrentPopup();
    }

    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
      ImGui::CloseCurrentPopup();
      clear();
    }

    ImGui::EndPopup();
  }

  if (open_preprocess_modal) {
    progress_modal->open<std::pair<gtsam_points::PointCloudCPU::Ptr, gtsam_points::PointCloudCPU::Ptr>>("preprocess", [this](guik::ProgressInterface& progress) {
      return preprocess(progress);
    });
  }

  auto preprocessed = progress_modal->run<std::pair<gtsam_points::PointCloudCPU::Ptr, gtsam_points::PointCloudCPU::Ptr>>("preprocess");
  if (preprocessed) {
    target = preprocessed->first;
    source = preprocessed->second;

    target_drawable = std::make_shared<glk::PointCloudBuffer>(target->points, target->size());
    source_drawable = std::make_shared<glk::PointCloudBuffer>(source->points, source->size());

    ImGui::OpenPopup("manual loop close");
    model_control->set_model_matrix((target_pose.inverse() * source_pose).cast<float>().matrix());
  }

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

    if (ImGui::DragFloat("fpfh_radius", &fpfh_radius, 0.01f, 0.01f, 100.0f)) {
      target->aux_attributes.erase("fpfh");
      source->aux_attributes.erase("fpfh");
    }

    ImGui::DragFloat("max_corr_dist", &max_correspondence_distance, 0.01f, 0.01f, 100.0f);
    ImGui::DragFloat("inf_scale", &information_scale, 0.0f, 1.0f, 10000.0f);

    bool open_align_global_modal = false;
    if (ImGui::Button("Align Global")) {
      open_align_global_modal = true;
    }

    bool open_align_modal = false;
    if (ImGui::Button("Align")) {
      open_align_modal = true;
    }

    if (open_align_global_modal) {
      progress_modal->open<std::shared_ptr<Eigen::Isometry3d>>("align", [this](guik::ProgressInterface& progress) { return align_global(progress); });
    }
    if (open_align_modal) {
      progress_modal->open<std::shared_ptr<Eigen::Isometry3d>>("align", [this](guik::ProgressInterface& progress) { return align(progress); });
    }
    auto align_result = progress_modal->run<std::shared_ptr<Eigen::Isometry3d>>("align");
    if (align_result) {
      model_control->set_model_matrix((*align_result)->cast<float>().matrix());
    }

    if (ImGui::Button("Create Factor")) {
      factor = create_factor();
      ImGui::CloseCurrentPopup();
      clear();
    }

    if (ImGui::Button("Cancel")) {
      ImGui::CloseCurrentPopup();
      clear();
    }

    ImGui::EndPopup();
  }
  return factor;
}

std::pair<gtsam_points::PointCloudCPU::Ptr, gtsam_points::PointCloudCPU::Ptr> ManualLoopCloseModal::preprocess(guik::ProgressInterface& progress) {
  logger->info("preprocessing maps");
  progress.set_title("Preprocessing maps");
  progress.set_maximum(target_submaps.size() + source_submaps.size());

  progress.set_text("Downsampling");
  const auto preprocess = [&](const std::vector<SubMap::ConstPtr>& submaps) {
    gtsam_points::iVox ivox(min_distance * 5.0);
    ivox.set_lru_horizon(1000000);
    ivox.voxel_insertion_setting().max_num_points_in_cell = 50;
    ivox.voxel_insertion_setting().min_sq_dist_in_cell = std::pow(min_distance, 2);
    for (const auto& submap : submaps) {
      progress.increment();
      auto transformed = gtsam_points::transform(submap->frame, submap->T_world_origin);
      ivox.insert(*transformed);
    }

    return std::make_shared<gtsam_points::PointCloudCPU>(ivox.voxel_points());
  };

  gtsam_points::PointCloudCPU::Ptr target;
  gtsam_points::PointCloudCPU::Ptr source;

  logger->info("preprocessing");
  target = preprocess(target_submaps);
  source = preprocess(source_submaps);

  gtsam_points::CovarianceEstimationParams cov_params;
  cov_params.num_threads = num_threads;

  logger->info("|target|={} |source|={}", target->size(), source->size());
  logger->info("estimating target normals and covariances");
  progress.set_text("Estimating covariances and normals");
  target->add_covs(gtsam_points::estimate_covariances(target->points, target->size(), cov_params));
  target->add_normals(gtsam_points::estimate_normals(target->points, target->covs, target->size(), num_threads));

  logger->info("estimating source normals and covariances");
  source->add_covs(gtsam_points::estimate_covariances(source->points, source->size(), cov_params));
  source->add_normals(gtsam_points::estimate_normals(source->points, source->covs, source->size(), num_threads));

  progress.set_text("Done");

  return {target, source};
}

std::shared_ptr<Eigen::Isometry3d> ManualLoopCloseModal::align_global(guik::ProgressInterface& progress) {
  logger->info("Aligning global maps");
  progress.set_title("Global registration");
  progress.set_maximum(10);

  progress.increment();
  logger->info("Creating KdTree");
  progress.set_text("Creating KdTree");
  gtsam_points::KdTree2<gtsam_points::PointCloud> target_tree(target, num_threads);
  progress.increment();
  gtsam_points::KdTree2<gtsam_points::PointCloud> source_tree(source, num_threads);

  gtsam_points::FPFHEstimationParams fpfh_params;
  fpfh_params.num_threads = num_threads;
  fpfh_params.search_radius = fpfh_radius;

  progress.increment();
  logger->info("Estimating target FPFH features");
  progress.set_text("Estimating target FPFH features");
  if (!target->aux_attributes.count("fpfh")) {
    const auto fpfh = gtsam_points::estimate_fpfh(target->points, target->normals, target->size(), target_tree, fpfh_params);
    target->add_aux_attribute("fpfh", fpfh);
  }

  progress.increment();
  logger->info("Estimating source FPFH features");
  progress.set_text("Estimating source FPFH features");
  if (!source->aux_attributes.count("fpfh")) {
    const auto fpfh = gtsam_points::estimate_fpfh(source->points, source->normals, source->size(), source_tree, fpfh_params);
    source->add_aux_attribute("fpfh", fpfh);
  }

  progress.increment();
  logger->info("Creating feature KdTree");
  progress.set_text("Creating feature KdTree");
  const auto target_fpfh = target->aux_attribute<gtsam_points::FPFHSignature>("fpfh");
  const auto source_fpfh = source->aux_attribute<gtsam_points::FPFHSignature>("fpfh");
  gtsam_points::KdTreeX<gtsam_points::FPFH_DIM> target_fpfh_tree(target_fpfh, target->size());
  progress.increment();
  gtsam_points::KdTreeX<gtsam_points::FPFH_DIM> source_fpfh_tree(source_fpfh, source->size());

  progress.increment();
  logger->info("Estimating transformation");
  progress.set_text("Estimating transformation");
  gtsam_points::GNCParams gnc_params;
  gnc_params.max_init_samples = 15000;
  gnc_params.reciprocal_check = false;
  gnc_params.tuple_check = true;
  gnc_params.max_num_tuples = 2500;
  gnc_params.verbose = true;
  gnc_params.num_threads = num_threads;
  auto result = gtsam_points::estimate_pose_gnc<gtsam_points::PointCloud>(*target, *source, target_fpfh, source_fpfh, target_tree, target_fpfh_tree, source_fpfh_tree, gnc_params);

  std::shared_ptr<Eigen::Isometry3d> trans(new Eigen::Isometry3d(result.T_target_source));
  return trans;
}

std::shared_ptr<Eigen::Isometry3d> ManualLoopCloseModal::align(guik::ProgressInterface& progress) {
  progress.set_title("Aligning frames");
  int num_iterations = 20;
  progress.set_maximum(num_iterations);

  progress.set_text("Creating graph");
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(0, gtsam::Pose3::Identity(), gtsam::noiseModel::Isotropic::Precision(6, 1e6));

  if (target->has_covs() && source->has_covs()) {
    auto factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(0, 1, target, source);
    factor->set_num_threads(num_threads);
    factor->set_max_correspondence_distance(max_correspondence_distance);
    graph.add(factor);
  } else {
    auto factor = gtsam::make_shared<gtsam_points::IntegratedICPFactor>(0, 1, target, source);
    factor->set_num_threads(num_threads);
    factor->set_max_correspondence_distance(max_correspondence_distance);
    graph.add(factor);

    num_iterations = 200;
    progress.set_maximum(num_iterations);
  }

  gtsam::Values values;
  values.insert(0, gtsam::Pose3::Identity());
  values.insert(1, gtsam::Pose3(model_control->model_matrix().cast<double>()));

  progress.set_text("Optimizing");
  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setMaxIterations(num_iterations);
  lm_params.callback = [&](const gtsam_points::LevenbergMarquardtOptimizationStatus& status, const gtsam::Values& values) {
    progress.increment();
    progress.set_text(fmt::format("Optimizing iter:{} error:{:.3f}", status.iterations, status.error));
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
  using gtsam::symbol_shorthand::X;

  const gtsam::Pose3 relative(model_control->model_matrix().cast<double>());

  if (target_submaps.size()) {
    const Eigen::Isometry3d T_target_source(relative.matrix());

    double min_distance = std::numeric_limits<double>::max();
    std::pair<SubMap::ConstPtr, SubMap::ConstPtr> best_pair;

    for (const auto& target : target_submaps) {
      for (const auto& source : source_submaps) {
        const double distance = (target->T_world_origin.translation() - T_target_source * source->T_world_origin.translation()).norm();
        if (distance < min_distance) {
          min_distance = distance;
          best_pair = std::make_pair(target, source);
        }
      }
    }

    const auto target_anchor = best_pair.first;
    const auto source_anchor = best_pair.second;
    logger->info("target_id:{} source_id:{} dist={}", target_anchor->id, source_anchor->id, min_distance);

    const Eigen::Isometry3d T_target_anchort = target_anchor->T_world_origin;
    const Eigen::Isometry3d T_source_anchors = source_anchor->T_world_origin;
    const Eigen::Isometry3d T_anchort_anchors = T_target_anchort.inverse() * T_target_source * T_source_anchors;

    return gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      X(target_anchor->id),
      X(source_anchor->id),
      gtsam::Pose3(T_anchort_anchors.matrix()),
      gtsam::noiseModel::Isotropic::Sigma(6, 1e-6));
  }

  gtsam::Values values;
  values.insert(0, gtsam::Pose3::Identity());
  values.insert(1, relative);

  auto factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(0, 1, target, source);
  factor->set_num_threads(num_threads);
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