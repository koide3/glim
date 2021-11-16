#include <glim/viewer/interactive_viewer.hpp>

#include <atomic>
#include <thread>
#include <glim/common/callbacks.hpp>
#include <glim/backend/sub_map.hpp>
#include <glim/backend/callbacks.hpp>
#include <glim/util/concurrent_vector.hpp>

#include <glim/viewer/interactive/manual_loop_close_modal.hpp>
#include <glim/viewer/interactive/bundle_adjustment_modal.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam_ext/factors/integrated_matching_cost_factor.hpp>
#include <gtsam_ext/factors/integrated_vgicp_factor_gpu.hpp>

#include <glk/thin_lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace glim {

using gtsam::symbol_shorthand::X;

InteractiveViewer::InteractiveViewer() {
  kill_switch = false;
  request_to_terminate = false;
  request_to_clear = false;

  coord_scale = 1.0f;
  sphere_scale = 1.0f;

  draw_points = true;
  draw_factors = true;
  draw_spheres = true;

  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  // CommonCallbacks::on_notification.add(std::bind(&InteractiveViewer::common_notification, this, _1, _2));
  GlobalMappingCallbacks::on_insert_submap.add(std::bind(&InteractiveViewer::globalmap_on_insert_submap, this, _1));
  GlobalMappingCallbacks::on_update_submaps.add(std::bind(&InteractiveViewer::globalmap_on_update_submaps, this, _1));
  GlobalMappingCallbacks::on_smoother_update.add(std::bind(&InteractiveViewer::globalmap_on_smoother_update, this, _1, _2, _3));

  thread = std::thread([this] { viewer_loop(); });
}

InteractiveViewer::~InteractiveViewer() {
  kill_switch = true;
  if (thread.joinable()) {
    thread.join();
  }
}

/**
 * @brief Main viewer loop
 */
void InteractiveViewer::viewer_loop() {
  auto viewer = guik::LightViewer::instance(Eigen::Vector2i(2560, 1440));
  viewer->enable_info_buffer();
  viewer->enable_vsync();
  viewer->register_ui_callback("selection", [this] { drawable_selection(); });
  viewer->register_ui_callback("on_click", [this] { on_click(); });
  viewer->register_ui_callback("context_menu", [this] { context_menu(); });
  viewer->register_ui_callback("run_modals", [this] { run_modals(); });

  viewer->add_drawable_filter("filter", [this](const std::string& name) {
    const auto starts_with = [](const std::string& name, const std::string& pattern) {
      return name.size() < pattern.size() ? false : std::equal(pattern.begin(), pattern.end(), name.begin());
    };

    if (!draw_points && starts_with(name, "submap_")) {
      return false;
    }
    if (!draw_factors && name == "factors") {
      return false;
    }
    if (!draw_spheres && starts_with(name, "sphere_")) {
      return false;
    }

    return true;
  });

  manual_loop_close_modal.reset(new ManualLoopCloseModal);
  bundle_adjustment_modal.reset(new BundleAdjustmentModal);

  setup_ui();

  while (!kill_switch) {
    if (!viewer->spin_once()) {
      request_to_terminate = true;
    }

    std::lock_guard<std::mutex> lock(invoke_queue_mutex);
    for (const auto& task : invoke_queue) {
      task();
    }
    invoke_queue.clear();
  }

  manual_loop_close_modal.reset();
  bundle_adjustment_modal.reset();
  guik::LightViewer::destroy();
}

/**
 * @brief Request to invoke a task on the GUI thread
 */
void InteractiveViewer::invoke(const std::function<void()>& task) {
  if (kill_switch) {
    return;
  }
  std::lock_guard<std::mutex> lock(invoke_queue_mutex);
  invoke_queue.push_back(task);
}

/**
 * @brief Drawable selection UI
 */
void InteractiveViewer::drawable_selection() {
  ImGui::Begin("Selection", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::Checkbox("Points", &draw_points);
  ImGui::Checkbox("Factors", &draw_factors);
  ImGui::Checkbox("Spheres", &draw_spheres);
  ImGui::End();
}

/**
 * @brief Click callback
 */
void InteractiveViewer::on_click() {
  ImGuiIO& io = ImGui::GetIO();
  if (io.WantCaptureMouse) {
    return;
  }

  const auto mouse_pos = ImGui::GetMousePos();
  if (!ImGui::IsMouseClicked(1)) {
    return;
  }

  auto viewer = guik::LightViewer::instance();
  right_clicked_info = viewer->pick_info(Eigen::Vector2i(mouse_pos.x, mouse_pos.y));
  const float depth = viewer->pick_depth(Eigen::Vector2i(mouse_pos.x, mouse_pos.y));
  right_clicked_pos = viewer->unproject(Eigen::Vector2i(mouse_pos.x, mouse_pos.y), depth);
}

/**
 * @brief Context menu
 */
void InteractiveViewer::context_menu() {
  if (ImGui::BeginPopupContextVoid("context menu")) {
    const PickType type = static_cast<PickType>(right_clicked_info[0]);

    if (type == PickType::FRAME) {
      const int frame_id = right_clicked_info[3];
      if (ImGui::MenuItem("Loop begin")) {
        manual_loop_close_modal->set_target(X(frame_id), submaps[frame_id]->frame, submap_poses[frame_id]);
      }
      if (ImGui::MenuItem("Loop end")) {
        manual_loop_close_modal->set_source(X(frame_id), submaps[frame_id]->frame, submap_poses[frame_id]);
      }
    }

    if (type == PickType::POINTS) {
      if (ImGui::MenuItem("Bundle adjustment (Plane)")) {
        bundle_adjustment_modal->set_frames(submaps, submap_poses, right_clicked_pos.cast<double>());
      }
    }

    ImGui::EndPopup();
  }
}

/**
 * @brief Run modals
 */
void InteractiveViewer::run_modals() {
  std::vector<gtsam::NonlinearFactor::shared_ptr> factors;
  factors.push_back(manual_loop_close_modal->run());
  factors.push_back(bundle_adjustment_modal->run());

  factors.erase(std::remove(factors.begin(), factors.end(), nullptr), factors.end());

  if (factors.size()) {
    new_factors.insert(factors);
    GlobalMappingCallbacks::request_to_optimize();
  }
}

/**
 * @brief Update viewer
 */
void InteractiveViewer::update_viewer() {
  auto viewer = guik::LightViewer::instance();

  Eigen::Vector2f z_range(0.0f, 0.0f);
  for (int i = 0; i < submaps.size(); i++) {
    const auto& submap = submaps[i];
    const Eigen::Affine3f submap_pose = submap_poses[i].cast<float>();

    auto drawable = viewer->find_drawable("submap_" + std::to_string(submap->id));
    if (drawable.first) {
      drawable.first->add("model_matrix", submap_pose.matrix());
    } else {
      const Eigen::Vector4i info(static_cast<int>(PickType::POINTS), 0, 0, submap->id);
      auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(submap->frame->points, submap->frame->size());
      viewer->update_drawable("submap_" + std::to_string(submap->id), cloud_buffer, guik::Rainbow(submap_pose).add("info_values", info));
    }

    const Eigen::Vector4i info(static_cast<int>(PickType::FRAME), 0, 0, submap->id);

    viewer->update_drawable(
      "coord_" + std::to_string(submap->id),
      glk::Primitives::coordinate_system(),
      guik::VertexColor(submap_pose * Eigen::UniformScaling<float>(coord_scale)).add("info_values", info));

    viewer->update_drawable(
      "sphere_" + std::to_string(submap->id),
      glk::Primitives::sphere(),
      guik::FlatColor(1.0f, 0.0f, 0.0f, 0.5f, submap_pose * Eigen::UniformScaling<float>(sphere_scale)).add("info_values", info).make_transparent());
  }

  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> factor_lines;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> factor_colors;
  factor_lines.reserve(global_factors.size() * 2);
  factor_colors.reserve(global_factors.size() * 2);

  const auto get_position = [this](const gtsam::Key key) -> Eigen::Vector3d {
    gtsam::Symbol symbol(key);
    switch (symbol.chr()) {
      case 'x':
        return submaps[symbol.index()]->T_world_origin.translation();
      case 'e': {
        const int right = symbol.index() % 2;
        const int submap_id = (symbol.index() - right) / 2;

        const auto& submap = submaps[submap_id];
        const auto& T_origin_endpoint = right ? submap->T_origin_endpoint_R : submap->T_origin_endpoint_L;
        const Eigen::Isometry3d T_world_endpoint = submap->T_world_origin * T_origin_endpoint;

        return T_world_endpoint.translation();
      }
    }

    std::cout << "warning: unknown symbol " << symbol << std::endl;
    return Eigen::Vector3d(0.0, 0.0, 0.0);
  };

  for (const auto& factor : global_factors) {
    FactorType type = std::get<0>(factor);
    factor_lines.push_back(get_position(std::get<1>(factor)).cast<float>());
    factor_lines.push_back(get_position(std::get<2>(factor)).cast<float>());

    Eigen::Vector4f color;
    switch (type) {
      case FactorType::MATCHING_COST:
        color = Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f);
        break;
      case FactorType::BETWEEN:
        color = Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0f);
        break;
      case FactorType::IMU:
        color = Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f);
        break;
    }

    factor_colors.push_back(color);
    factor_colors.push_back(color);
  }

  viewer->update_drawable("factors", std::make_shared<glk::ThinLines>(factor_lines, factor_colors), guik::VertexColor());
}

/**
 * @brief New submap insertion callback
 */
void InteractiveViewer::globalmap_on_insert_submap(const SubMap::ConstPtr& submap) {
  std::shared_ptr<Eigen::Isometry3d> pose(new Eigen::Isometry3d(submap->T_world_origin));
  invoke([this, submap, pose] {
    submap_poses.push_back(*pose);
    submaps.push_back(submap);
    update_viewer();
  });
}

/**
 * @brief Submap pose update callback
 */
void InteractiveViewer::globalmap_on_update_submaps(const std::vector<SubMap::Ptr>& updated_submaps) {
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses(updated_submaps.size());
  std::transform(updated_submaps.begin(), updated_submaps.end(), poses.begin(), [](const SubMap::ConstPtr& submap) { return submap->T_world_origin; });

  invoke([this, poses] {
    for (int i = 0; i < std::min(submaps.size(), poses.size()); i++) {
      submap_poses[i] = poses[i];
    }
    update_viewer();
  });
}

/**
 * @brief Smoother update callback
 */
void InteractiveViewer::globalmap_on_smoother_update(gtsam_ext::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) {
  auto factors = this->new_factors.get_all_and_clear();
  new_factors.add(factors);

  std::vector<std::tuple<FactorType, gtsam::Key, gtsam::Key>> inserted_factors;

  for (const auto& factor : new_factors) {
    if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor)) {
      inserted_factors.push_back(std::make_tuple(FactorType::BETWEEN, factor->keys()[0], factor->keys()[1]));
    }
    if (boost::dynamic_pointer_cast<gtsam_ext::IntegratedMatchingCostFactor>(factor)) {
      inserted_factors.push_back(std::make_tuple(FactorType::MATCHING_COST, factor->keys()[0], factor->keys()[1]));
    }
    if (boost::dynamic_pointer_cast<gtsam_ext::IntegratedVGICPFactorGPU>(factor)) {
      inserted_factors.push_back(std::make_tuple(FactorType::MATCHING_COST, factor->keys()[0], factor->keys()[1]));
    }
    if (boost::dynamic_pointer_cast<gtsam::ImuFactor>(factor)) {
      inserted_factors.push_back(std::make_tuple(FactorType::IMU, factor->keys()[0], factor->keys()[2]));
    }
  }

  invoke([this, inserted_factors] { global_factors.insert(global_factors.end(), inserted_factors.begin(), inserted_factors.end()); });
}

bool InteractiveViewer::ok() const {
  return !request_to_terminate;
}

void InteractiveViewer::wait() {
  while (!request_to_terminate) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  stop();
}

void InteractiveViewer::stop() {
  std::this_thread::sleep_for(std::chrono::seconds(1));

  kill_switch = true;
  if (thread.joinable()) {
    thread.join();
  }
}

void InteractiveViewer::clear() {
  std::lock_guard<std::mutex> lock(invoke_queue_mutex);
  invoke_queue.clear();

  submaps.clear();
  submap_poses.clear();
  global_factors.clear();
  new_factors.clear();

  guik::LightViewer::instance()->clear_drawables();
}

}  // namespace glim
