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
#include <gtsam_points/registration/ransac.hpp>
#include <gtsam_points/registration/graduated_non_convexity.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>

#include <glim/util/convert_to_string.hpp>
#include <glim/common/cloud_covariance_estimation.hpp>

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

  seed = 53123;
  min_distance = 0.5f;
  fpfh_radius = 5.0f;
  global_registration_type = 0;

  ransac_max_iterations = 5000;
  ransac_early_stop_rate = 0.9;
  ransac_inlier_voxel_resolution = 1.0;
  global_registration_4dof = true;

  gnc_max_samples = 10000;

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

bool ManualLoopCloseModal::is_target_set() const {
  return target_key != -1 && target && target_drawable;
}

bool ManualLoopCloseModal::is_source_set() const {
  return source_key != -1 && source && source_drawable;
}

void ManualLoopCloseModal::set_target(const gtsam::Key target_key, const gtsam_points::PointCloud::ConstPtr& target, const Eigen::Isometry3d& target_pose) {
  this->target_key = target_key;
  this->target_pose = target_pose;
  this->target = gtsam_points::PointCloudCPU::clone(*target);

  // Gravity (Z-axis) alignment
  Eigen::Isometry3d T_world_local = Eigen::Isometry3d::Identity();
  T_world_local.linear() = target_pose.linear();
  gtsam_points::transform_inplace(this->target, T_world_local);

  this->target_drawable = std::make_shared<glk::PointCloudBuffer>(this->target->points, this->target->size());
}

void ManualLoopCloseModal::set_source(const gtsam::Key source_key, const gtsam_points::PointCloud::ConstPtr& source, const Eigen::Isometry3d& source_pose) {
  this->source_key = source_key;
  this->source_pose = source_pose;
  this->source = gtsam_points::PointCloudCPU::clone(*source);

  // Gravity (Z-axis) alignment
  Eigen::Isometry3d T_world_local = Eigen::Isometry3d::Identity();
  T_world_local.linear() = source_pose.linear();
  gtsam_points::transform_inplace(this->source, T_world_local);

  this->source_drawable = std::make_shared<glk::PointCloudBuffer>(this->source->points, this->source->size());
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
  target_fpfh_tree = nullptr;
  source_fpfh_tree = nullptr;
  target_drawable = nullptr;
  source_drawable = nullptr;
  target_submaps.clear();
  source_submaps.clear();
}

gtsam::NonlinearFactor::shared_ptr ManualLoopCloseModal::run() {
  gtsam::NonlinearFactor::shared_ptr factor;

  if (request_to_open && target && source) {
    // Setup for submap vs submap loop closure
    Eigen::Isometry3d init_T_target_source = Eigen::Isometry3d::Identity();

    Eigen::Isometry3d R_world_target = Eigen::Isometry3d::Identity();
    R_world_target.linear() = target_pose.linear();

    Eigen::Isometry3d R_world_source = Eigen::Isometry3d::Identity();
    R_world_source.linear() = source_pose.linear();

    init_T_target_source.translation() = (R_world_target * target_pose.inverse() * source_pose * R_world_source.inverse()).translation();
    model_control->set_model_matrix(init_T_target_source);

    // Open the manual loop close modal
    ImGui::OpenPopup("manual loop close");
  } else if (request_to_open && target_submaps.size() && source_submaps.size()) {
    // Setup for session vs session loop closure
    model_control->set_model_matrix(Eigen::Matrix4f::Identity().eval());
    ImGui::OpenPopup("preprocess maps");
  }
  request_to_open = false;

  bool open_preprocess_modal = false;  // Request to open session preprocessing progress modal
  // Preprocess parameter setting modal
  if (ImGui::BeginPopupModal("preprocess maps", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("Set default parameters:");
    show_note("Select default parameters for minimum distance for downsampling and neighbor search radius for FPFH extraction.");

    ImGui::SameLine();
    if (ImGui::Button("Indoor")) {
      min_distance = 0.25f;
      fpfh_radius = 2.5f;
      max_correspondence_distance = 1.0f;
    }
    ImGui::SameLine();
    if (ImGui::Button("Outdoor")) {
      min_distance = 0.5f;
      fpfh_radius = 5.0f;
      max_correspondence_distance = 3.0f;
    }

    ImGui::DragFloat("Min distance", &min_distance, 0.01f, 0.01f, 100.0f) || show_note("Minimum distance between points for downsampling.");

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

  // Open the preprocessing progress modal
  if (open_preprocess_modal) {
    progress_modal->open<std::pair<gtsam_points::PointCloudCPU::Ptr, gtsam_points::PointCloudCPU::Ptr>>("preprocess", [this](guik::ProgressInterface& progress) {
      return preprocess_maps(progress);
    });
  }

  // Run the preprocessing progress modal and get the preprocessed point clouds
  auto preprocessed = progress_modal->run<std::pair<gtsam_points::PointCloudCPU::Ptr, gtsam_points::PointCloudCPU::Ptr>>("preprocess");
  if (preprocessed) {
    target = preprocessed->first;
    source = preprocessed->second;

    target_drawable = std::make_shared<glk::PointCloudBuffer>(target->points, target->size());
    source_drawable = std::make_shared<glk::PointCloudBuffer>(source->points, source->size());

    model_control->set_model_matrix(Eigen::Matrix4f::Identity().eval());

    // Open the manual loop close modal
    ImGui::OpenPopup("manual loop close");
  }

  // Manual loop close modal
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

    ImGui::Separator();

    /*** Global registration ***/

    ImGui::Combo("Global registration type", &global_registration_type, "RANSAC\0GNC\0");

    if (ImGui::DragFloat("fpfh_radius", &fpfh_radius, 0.01f, 0.01f, 100.0f) || show_note("Neighbor search radius for FPFH extraction.\n~2.5m for indoors, ~5.0m for outdoors.")) {
      target->aux_attributes.erase("fpfh");
      source->aux_attributes.erase("fpfh");
    }
    if (target->aux_attributes.count("fpfh")) {
      ImGui::SameLine();
      ImGui::Text("[Cached]");
    }

    switch (global_registration_type) {
      case 0:  // RANSAC
        ImGui::DragInt("max_iterations", &ransac_max_iterations, 100, 1, 100000) || show_note("Maximum number of RANSAC iterations.");
        ImGui::DragFloat("inlier_voxel_resolution", &ransac_inlier_voxel_resolution, 0.01f, 0.01f, 100.0f) || show_note("Resolution of voxelmap used for inlier check.");
        break;
      case 1:
        ImGui::DragInt("max_samples", &gnc_max_samples, 100, 1, 100000) || show_note("Maximum number of feature samples for GNC.");
        break;
    }
    ImGui::Checkbox("4dof", &global_registration_4dof) || show_note("Use 4DoF (XYZ + RZ) estimation instead of 6DoF (SE3).");

    bool open_align_global_modal = false;
    if (ImGui::Button("Run global registration")) {
      open_align_global_modal = true;
    }

    /*** Fine registration ***/

    ImGui::Separator();
    ImGui::DragFloat("max_corr_dist", &max_correspondence_distance, 0.01f, 0.01f, 100.0f) || show_note("Maximum correspondence distance for scan matching.");
    ImGui::DragFloat("inf_scale", &information_scale, 0.0f, 1.0f, 10000.0f) || show_note("Information scale for loop factor.");

    bool open_align_modal = false;
    if (ImGui::Button("Run fine registration")) {
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

    /*** Factor creation ***/

    ImGui::Separator();
    if (ImGui::Button("Create Factor") || show_note("Create a loop factor with the estimated transformation.")) {
      factor = create_factor();
      ImGui::CloseCurrentPopup();
      clear();
    }

    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
      ImGui::CloseCurrentPopup();
      clear();
    }

    ImGui::EndPopup();
  }
  return factor;
}

std::pair<gtsam_points::PointCloudCPU::Ptr, gtsam_points::PointCloudCPU::Ptr> ManualLoopCloseModal::preprocess_maps(guik::ProgressInterface& progress) {
  logger->info("preprocessing maps");
  progress.set_title("Preprocessing maps");
  progress.set_maximum(target_submaps.size() + source_submaps.size());

  progress.set_text("Downsampling");
  const auto preprocess = [&](const std::vector<SubMap::ConstPtr>& submaps) {
    logger->info("Downsampling");
    gtsam_points::iVox ivox(min_distance * 5.0);
    ivox.set_lru_horizon(1000000);
    ivox.voxel_insertion_setting().max_num_points_in_cell = 50;
    ivox.voxel_insertion_setting().min_sq_dist_in_cell = std::pow(min_distance, 2);
    for (const auto& submap : submaps) {
      progress.increment();
      auto transformed = gtsam_points::transform(submap->frame, submap->T_world_origin);
      ivox.insert(*transformed);
    }

    auto points = std::make_shared<gtsam_points::PointCloudCPU>(ivox.voxel_points());

    logger->info("Finding neighbors");
    const int k = 10;
    gtsam_points::KdTree2<gtsam_points::PointCloud> tree(points);
    std::vector<int> neighbors(points->size() * k);

#pragma omp parallel for num_threads(num_threads) schedule(guided, 8)
    for (int i = 0; i < points->size(); i++) {
      std::vector<size_t> k_indices(k);
      std::vector<double> k_sq_dists(k);
      tree.knn_search(points->points[i].data(), k, k_indices.data(), k_sq_dists.data());
      std::copy(k_indices.begin(), k_indices.end(), neighbors.begin() + i * k);
    }

    logger->info("Estimate covariances");
    glim::CloudCovarianceEstimation covest(num_threads);
    covest.estimate(points->points_storage, neighbors, points->normals_storage, points->covs_storage);

    points->normals = points->normals_storage.data();
    points->covs = points->covs_storage.data();

    return points;
  };

  logger->info("preprocessing");
  gtsam_points::PointCloudCPU::Ptr target = preprocess(target_submaps);
  gtsam_points::PointCloudCPU::Ptr source = preprocess(source_submaps);

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
  auto target_tree = std::make_shared<gtsam_points::KdTree2<gtsam_points::PointCloud>>(target);
  progress.increment();
  auto source_tree = std::make_shared<gtsam_points::KdTree2<gtsam_points::PointCloud>>(source);

  gtsam_points::FPFHEstimationParams fpfh_params;
  fpfh_params.num_threads = num_threads;
  fpfh_params.search_radius = fpfh_radius;

  progress.increment();
  if (!target->aux_attributes.count("fpfh")) {
    if (!target->has_normals()) {
      logger->info("Estimating target normals");
      progress.set_text("Estimating target normals");
      target->add_normals(gtsam_points::estimate_normals(target->points, target->covs, target->size(), num_threads));
    }

    logger->info("Estimating target FPFH features");
    progress.set_text("Estimating target FPFH features");
    const auto fpfh = gtsam_points::estimate_fpfh(target->points, target->normals, target->size(), *target_tree, fpfh_params);
    target->add_aux_attribute("fpfh", fpfh);

    logger->info("Constructing target FPFH KdTree");
    progress.set_text("Constructing target FPFH KdTree");
    const auto target_fpfh = target->aux_attribute<gtsam_points::FPFHSignature>("fpfh");
    target_fpfh_tree = std::make_shared<gtsam_points::KdTreeX<gtsam_points::FPFH_DIM>>(target_fpfh, target->size());
  }

  progress.increment();
  if (!source->aux_attributes.count("fpfh")) {
    if (!source->has_normals()) {
      logger->info("Estimating source normals");
      progress.set_text("Estimating source normals");
      source->add_normals(gtsam_points::estimate_normals(source->points, source->covs, source->size(), num_threads));
    }

    logger->info("Estimating source FPFH features");
    progress.set_text("Estimating source FPFH features");
    const auto fpfh = gtsam_points::estimate_fpfh(source->points, source->normals, source->size(), *source_tree, fpfh_params);
    source->add_aux_attribute("fpfh", fpfh);

    logger->info("Constructing source FPFH KdTree");
    progress.set_text("Constructing source FPFH KdTree");
    const auto source_fpfh = source->aux_attribute<gtsam_points::FPFHSignature>("fpfh");
    source_fpfh_tree = std::make_shared<gtsam_points::KdTreeX<gtsam_points::FPFH_DIM>>(source_fpfh, source->size());
  }

  const auto target_fpfh = target->aux_attribute<gtsam_points::FPFHSignature>("fpfh");
  const auto source_fpfh = source->aux_attribute<gtsam_points::FPFHSignature>("fpfh");

  progress.increment();

  gtsam_points::RegistrationResult result;

  if (global_registration_type == 0) {
    logger->info("Estimating transformation RANSAC (seed={})", seed);
    progress.set_text("Estimating transformation RANSAC");

    gtsam_points::RANSACParams ransac_params;
    ransac_params.max_iterations = ransac_max_iterations;
    ransac_params.early_stop_inlier_rate = ransac_early_stop_rate;
    ransac_params.inlier_voxel_resolution = ransac_inlier_voxel_resolution;
    ransac_params.dof = global_registration_4dof ? 4 : 6;
    ransac_params.seed = (seed += 4322);
    ransac_params.num_threads = num_threads;

    result = gtsam_points::estimate_pose_ransac(*target, *source, target_fpfh, source_fpfh, *target_tree, *target_fpfh_tree, ransac_params);

  } else {
    logger->info("Estimating transformation GNC (seed={})", seed);
    progress.set_text("Estimating transformation (GNC)");

    gtsam_points::GNCParams gnc_params;
    gnc_params.max_init_samples = gnc_max_samples;
    gnc_params.reciprocal_check = true;
    gnc_params.tuple_check = false;
    gnc_params.max_num_tuples = 5000;
    gnc_params.dof = global_registration_4dof ? 4 : 6;
    gnc_params.seed = (seed += 4322);
    gnc_params.num_threads = num_threads;

    result = gtsam_points::estimate_pose_gnc(*target, *source, target_fpfh, source_fpfh, *target_tree, *target_fpfh_tree, *source_fpfh_tree, gnc_params);
  }

  logger->info("Registration result");
  logger->info("T_target_source={}", convert_to_string(result.T_target_source));
  logger->info("inlier_rate={}", result.inlier_rate);

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

  if (target_submaps.size()) {
    const Eigen::Isometry3d T_target_source(model_control->model_matrix().cast<double>());

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

  const gtsam::Pose3 T_target_source(model_control->model_matrix().cast<double>());

  gtsam::Values values;
  values.insert(0, gtsam::Pose3::Identity());
  values.insert(1, T_target_source);

  auto factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(0, 1, target, source);
  factor->set_num_threads(num_threads);
  factor->set_max_correspondence_distance(max_correspondence_distance);

  const auto linearized = factor->linearize(values);
  const auto H = linearized->hessianBlockDiagonal()[1];

  // Cancel out the gravity alignment
  const gtsam::Pose3 T_gb_b = gtsam::Pose3(gtsam::Rot3(source_pose.linear()), gtsam::Vector3::Zero());
  const gtsam::Pose3 T_ga_a = gtsam::Pose3(gtsam::Rot3(target_pose.linear()), gtsam::Vector3::Zero());
  const gtsam::Pose3 T_a_b = T_ga_a.inverse() * T_target_source * T_gb_b;

  return gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(target_key, source_key, T_a_b, gtsam::noiseModel::Gaussian::Information(information_scale * H));
}

void ManualLoopCloseModal::draw_canvas() {
  canvas->bind();
  canvas->shader->set_uniform("color_mode", guik::ColorMode::VERTEX_COLOR);
  canvas->shader->set_uniform("model_matrix", Eigen::Matrix4f::Identity().eval());

  glk::Primitives::coordinate_system()->draw(*canvas->shader);

  canvas->shader->set_uniform("color_mode", guik::ColorMode::FLAT_COLOR);
  canvas->shader->set_uniform("material_color", Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
  canvas->shader->set_uniform("model_matrix", Eigen::Matrix4f::Identity().eval());

  target_drawable->draw(*canvas->shader);

  canvas->shader->set_uniform("material_color", Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f));
  canvas->shader->set_uniform("model_matrix", model_control->model_matrix());

  source_drawable->draw(*canvas->shader);

  canvas->unbind();
}

bool ManualLoopCloseModal::show_note(const std::string& note) {
  if (ImGui::IsItemHovered()) {
    ImGui::BeginTooltip();
    ImGui::Text("%s", note.c_str());
    ImGui::EndTooltip();
  }
  return false;
}

}  // namespace glim