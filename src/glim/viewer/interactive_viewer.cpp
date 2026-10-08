#include <glim/viewer/interactive_viewer.hpp>

#include <algorithm>
#include <atomic>
#include <regex>
#include <thread>
#include <spdlog/spdlog.h>
#include <glim/mapping/sub_map.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/mapping/graph_edit.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/util/concurrent_vector.hpp>
#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/trajectory_manager.hpp>

#include <glim/viewer/interactive/manual_loop_close_modal.hpp>
#include <glim/viewer/interactive/bundle_adjustment_modal.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam_points/config.hpp>
#include <gtsam_points/factors/integrated_matching_cost_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_points/optimizers/isam2_result_ext.hpp>

#include <glk/thin_lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/spdlog_sink.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace glim {
using gtsam::symbol_shorthand::X;

namespace {

std::vector<std::tuple<InteractiveViewer::FactorType, std::uint64_t, std::uint64_t>> extract_display_factors(const gtsam::NonlinearFactorGraph& factors) {
  std::vector<std::tuple<InteractiveViewer::FactorType, std::uint64_t, std::uint64_t>> display_factors;

  for (const auto& factor : factors) {
    if (dynamic_cast<gtsam::BetweenFactor<gtsam::Pose3>*>(factor.get())) {
      display_factors.emplace_back(InteractiveViewer::FactorType::BETWEEN, factor->keys()[0], factor->keys()[1]);
    }
    if (dynamic_cast<gtsam_points::IntegratedMatchingCostFactor*>(factor.get())) {
      display_factors.emplace_back(InteractiveViewer::FactorType::MATCHING_COST, factor->keys()[0], factor->keys()[1]);
    }
#ifdef GTSAM_POINTS_USE_CUDA
    if (dynamic_cast<gtsam_points::IntegratedVGICPFactorGPU*>(factor.get())) {
      display_factors.emplace_back(InteractiveViewer::FactorType::MATCHING_COST, factor->keys()[0], factor->keys()[1]);
    }
#endif
    if (dynamic_cast<gtsam::ImuFactor*>(factor.get())) {
      display_factors.emplace_back(InteractiveViewer::FactorType::IMU, factor->keys()[0], factor->keys()[2]);
    }
  }

  return display_factors;
}

}  // namespace

InteractiveViewer::InteractiveViewer() : logger(create_module_logger("viewer")) {
  glim::Config config(glim::GlobalConfig::get_config_path("config_viewer"));

  kill_switch = false;
  request_to_terminate = false;
  request_to_clear = false;

#ifdef _OPENMP
  num_threads = omp_get_max_threads();
#else
  num_threads = 1;
#endif

  color_mode = 0;
  coord_scale = 1.0f;
  sphere_scale = 0.5f;

  draw_current = true;
  draw_traj = false;
  draw_points = true;
  draw_factors = true;
  draw_spheres = true;

  min_overlap = 0.2f;
  cont_optimize = false;

  needs_session_merge = false;
  current_graph_edit_state = GraphEditState::IDLE;
  prune_selection_mode = false;
  show_submap_pruning_window = false;
  hide_selected_submaps = false;
  prune_source_session = false;
  show_other_session = false;
  session_merge_in_progress = false;
  prune_range_start = -1;
  prune_range_end = -1;
  prune_start_input = 0;
  prune_end_input = 0;
  candidate_orphaned_subgraph_count = 0;

  enable_partial_rendering = config.param("interactive_viewer", "enable_partial_rendering", false);
  partial_rendering_budget = config.param("interactive_viewer", "partial_rendering_budget", 1024);

  points_alpha = config.param("interactive_viewer", "points_alpha", 1.0);
  factors_alpha = config.param("interactive_viewer", "factors_alpha", 1.0);

  point_size = config.param("interactive_viewer", "point_size", 0.025);
  point_size_metric = config.param("interactive_viewer", "point_size_metric", false);
  point_shape_circle = config.param("interactive_viewer", "point_shape_circle", true);

  z_range = config.param("interactive_viewer", "default_z_range", Eigen::Vector2d(-2.0, 4.0)).cast<float>();

  trajectory.reset(new TrajectoryManager);

  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  OdometryEstimationCallbacks::on_new_frame.add(std::bind(&InteractiveViewer::odometry_on_new_frame, this, _1));
  GlobalMappingCallbacks::on_insert_submap.add(std::bind(&InteractiveViewer::globalmap_on_insert_submap, this, _1));
  GlobalMappingCallbacks::on_update_submaps.add(std::bind(&InteractiveViewer::globalmap_on_update_submaps, this, _1));
  GlobalMappingCallbacks::on_smoother_update.add(std::bind(&InteractiveViewer::globalmap_on_smoother_update, this, _1, _2, _3));
  GlobalMappingCallbacks::on_smoother_update_result.add(std::bind(&InteractiveViewer::globalmap_on_smoother_update_result, this, _1, _2));
  GlobalMappingCallbacks::on_graph_edit_state_changed.add(std::bind(&InteractiveViewer::globalmap_on_graph_edit_state_changed, this, _1, _2));
  GlobalMappingCallbacks::on_candidate_graph_updated.add(std::bind(&InteractiveViewer::globalmap_on_candidate_graph_updated, this, _1));

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
  viewer->shader_setting().add("z_range", z_range);

  viewer->shader_setting().set_point_size(point_size);

  if (point_size_metric) {
    viewer->shader_setting().set_point_scale_metric();
  }

  if (point_shape_circle) {
    viewer->shader_setting().set_point_shape_circle();
  }

  if (enable_partial_rendering) {
    viewer->enable_partial_rendering(0.1);
    viewer->shader_setting().add("dynamic_object", 1);
  }

  viewer->register_ui_callback("selection", [this] { drawable_selection(); });
  viewer->register_ui_callback("submap_pruning", [this] { draw_submap_pruning_window(); });
  viewer->register_ui_callback("on_click", [this] { on_click(); });
  viewer->register_ui_callback("context_menu", [this] { context_menu(); });
  viewer->register_ui_callback("run_modals", [this] { run_modals(); });
  viewer->register_ui_callback("logging", guik::create_logger_ui(glim::get_ringbuffer_sink(), 0.5));

  viewer->register_drawable_filter("filter", [this](const std::string& name) {
    const auto starts_with = [](const std::string& name, const std::string& pattern) {
      return name.size() < pattern.size() ? false : std::equal(pattern.begin(), pattern.end(), name.begin());
    };

    if (!draw_current && name == "current") {
      return false;
    }

    if (!draw_traj && starts_with(name, "traj")) {
      return false;
    }

    if (!draw_points && starts_with(name, "submap_")) {
      return false;
    }
    if (!draw_factors && name == "factors") {
      return false;
    }
    if (!draw_spheres && starts_with(name, "sphere_")) {
      return false;
    }

    for (const auto* prefix : {"submap_", "coord_", "sphere_"}) {
      if (starts_with(name, prefix)) {
        const int submap_id = std::stoi(name.substr(std::char_traits<char>::length(prefix)));
        const bool other_session = show_submap_pruning_window && !show_other_session && current_graph_edit_state.load() == GraphEditState::SESSION_MERGE_PENDING &&
                                   is_source_submap(submap_id) != prune_source_session;
        if (is_unavailable_submap(submap_id) || (hide_selected_submaps && is_selected_for_pruning(submap_id)) || other_session) {
          return false;
        }
      }
    }

    return true;
  });

  manual_loop_close_modal.reset(new ManualLoopCloseModal(logger, num_threads));
  bundle_adjustment_modal.reset(new BundleAdjustmentModal);

  setup_ui();

  logger->info("Starting interactive viewer");

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
  if (!ImGui::Begin("Selection", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    return;
  }

  const auto show_note = [](const std::string& text) {
    if (ImGui::IsItemHovered()) {
      ImGui::BeginTooltip();
      ImGui::Text("%s", text.c_str());
      ImGui::EndTooltip();
    }
    return false;
  };
  std::vector<const char*> color_modes = {"RAINBOW", "INTENSITY", "SESSION"};
  if (ImGui::Combo("ColorMode", &color_mode, color_modes.data(), color_modes.size())) {
    update_viewer();
  }
  show_note("Color mode for rendering submaps.\n- RAINBOW=Altitude encoding color\n- INTENSITY=Point intensity\n- SESSION=Session ID");

  ImGui::Checkbox("Trajectory", &draw_traj);
  ImGui::SameLine();
  ImGui::Checkbox("Submaps", &draw_points);

  ImGui::Checkbox("Factors", &draw_factors);
  ImGui::SameLine();
  ImGui::Checkbox("Spheres", &draw_spheres);

  ImGui::Checkbox("Current scan", &draw_current);

  if (ImGui::BeginMenu("Display settings")) {
    bool do_update_viewer = false;

    do_update_viewer |= ImGui::DragFloat("coord scale", &coord_scale, 0.01f, 0.01f, 100.0f);
    show_note("Submap coordinate system maker scale.");

    do_update_viewer |= ImGui::DragFloat("sphere scale", &sphere_scale, 0.01f, 0.01f, 100.0f);
    show_note("Submap selection sphere maker scale.");

    auto viewer = guik::viewer();
    if (ImGui::Checkbox("Cumulative rendering", &enable_partial_rendering)) {
      if (enable_partial_rendering && !viewer->partial_rendering_enabled()) {
        viewer->enable_partial_rendering(1e-1);
        viewer->shader_setting().add("dynamic_object", 1);
      } else {
        viewer->disable_partial_rendering();
      }

      // Update existing submap buffers
      for (int i = 0;; i++) {
        auto found = viewer->find_drawable("submap_" + std::to_string(i));
        if (!found.first) {
          break;
        }

        auto cb = std::dynamic_pointer_cast<const glk::PointCloudBuffer>(found.second);
        auto cloud_buffer = std::const_pointer_cast<glk::PointCloudBuffer>(cb);  // !!

        if (enable_partial_rendering) {
          cloud_buffer->enable_partial_rendering(partial_rendering_budget);
          found.first->add("dynamic_object", 0).make_transparent();
        } else {
          cloud_buffer->disable_partial_rendering();
          found.first->add("dynamic_object", 1);
        }
      }
    }

    ImGui::SameLine();
    ImGui::SetNextItemWidth(60);
    ImGui::DragInt("Budget", &partial_rendering_budget, 1, 1, 1000000);

    ImGui::EndMenu();

    if (do_update_viewer) {
      update_viewer();
    }
  }

  ImGui::Separator();
  ImGui::DragFloat("Min overlap", &min_overlap, 0.01f, 0.01f, 1.0f);
  show_note("Minimum overlap ratio for finding overlapping submaps.");

  const GraphEditState edit_state = current_graph_edit_state.load();
  const bool merge_pending = edit_state == GraphEditState::SESSION_MERGE_PENDING || needs_session_merge.load();
  const bool graph_editing = edit_state != GraphEditState::IDLE || needs_session_merge.load();

  ImGui::BeginDisabled(merge_pending);
  if (ImGui::Button("Find overlapping submaps") || show_note("Find overlapping submaps and create matching cost factors between them.")) {
    logger->info("finding overlapping submaps...");
    GlobalMappingCallbacks::request_to_find_overlapping_submaps(min_overlap);
  }
  ImGui::EndDisabled();

  ImGui::BeginDisabled(graph_editing);
  if (ImGui::Button("Recover graph") || show_note("Detect and fix corrupted graph.")) {
    logger->info("recovering graph...");
    GlobalMappingCallbacks::request_to_recover();
  }
  ImGui::EndDisabled();

  if (edit_state == GraphEditState::SESSION_MERGE_PENDING && !submaps.empty()) {
    ImGui::Separator();

    ImGui::BeginDisabled(session_merge_in_progress);
    if (ImGui::BeginTable("pruning", 2, ImGuiTableFlags_SizingFixedFit)) {
      const auto pruning_row = [this](const char* label, bool source, const std::vector<SubmapRange>& ranges) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        if (ImGui::Button(label)) {
          prune_source_session = source;
          show_submap_pruning_window = true;
          prune_selection_mode = true;
          show_other_session = false;
          prune_range_start = -1;
          prune_range_end = -1;
          prune_start_input = -1;
          prune_end_input = -1;
        }
        ImGui::TableNextColumn();
        ImGui::Text("%d selected", count_selected_submaps(ranges));
      };

      pruning_row("Prune current map...", false, normalized_target_prune_ranges);
      pruning_row("Prune incoming map...", true, normalized_source_prune_ranges);
      ImGui::EndTable();
    }

    ImGui::Separator();
    ImGui::Spacing();

    const bool target_empty = !has_remaining_submaps(false);
    const bool source_empty = !has_remaining_submaps(true);
    ImGui::BeginDisabled(target_empty || source_empty);
    if (ImGui::Button("Merge sessions") || show_note("Align and merge the lastly loaded session with the active graph.")) {
      begin_session_merge();
    }
    ImGui::EndDisabled();
    if (target_empty) {
      ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "Current map must retain at least one active submap.");
    } else if (source_empty) {
      ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "Incoming map must retain at least one active submap.");
    }
    ImGui::EndDisabled();
  }

  if (edit_state == GraphEditState::CANDIDATE_EDITING) {
    ImGui::Separator();
    if (candidate_orphaned_subgraph_count > 0) {
      ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.0f, 1.0f), "Graph has %zu orphaned subgraph(s).", candidate_orphaned_subgraph_count);
      ImGui::TextWrapped("Add loop closures or find overlapping submaps to connect the graph.");
    } else if (!candidate_diagnostic.empty()) {
      ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "Candidate graph could not be committed.");
      ImGui::TextWrapped("%s", candidate_diagnostic.c_str());
    }
  }

  ImGui::BeginDisabled(graph_editing);
  if (ImGui::Button("Optimize")) {
    logger->info("optimizing...");
    GlobalMappingCallbacks::request_to_optimize();
  }
  if (!graph_editing) {
    show_note("Optimize the graph.");
  }

  ImGui::SameLine();
  ImGui::Checkbox("##Cont optimize", &cont_optimize);
  if (graph_editing) {
    cont_optimize = false;
  } else if (cont_optimize) {
    GlobalMappingCallbacks::request_to_optimize();
  }
  if (!graph_editing) {
    show_note("Continuously optimize the graph.");
  }
  ImGui::EndDisabled();

  ImGui::End();
}

void InteractiveViewer::draw_submap_pruning_window() {
  if (!show_submap_pruning_window) {
    return;
  }

  if (current_graph_edit_state != GraphEditState::SESSION_MERGE_PENDING || session_merge_in_progress) {
    show_submap_pruning_window = false;
    prune_selection_mode = false;
    return;
  }

  ImGui::SetNextWindowSize(ImVec2(470.0f, 300.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("Submap Pruning", &show_submap_pruning_window)) {
    ImGui::End();
    if (!show_submap_pruning_window) {
      prune_selection_mode = false;
    }
    return;
  }

  auto& requested_prune_ranges = prune_source_session ? requested_source_prune_ranges : requested_target_prune_ranges;
  const auto& normalized_prune_ranges = prune_source_session ? normalized_source_prune_ranges : normalized_target_prune_ranges;

  ImGui::TextWrapped("Right-click a submap in the %s map to set range endpoints.", prune_source_session ? "incoming" : "current");

  ImGui::SetNextItemWidth(80.0f);
  if (ImGui::InputInt("Start", &prune_start_input)) {
    prune_range_start = -1;
  }
  ImGui::SameLine();
  ImGui::SetNextItemWidth(80.0f);
  if (ImGui::InputInt("End", &prune_end_input)) {
    prune_range_end = -1;
  }
  ImGui::SameLine();
  if (ImGui::Button("Add")) {
    if (add_prune_range(prune_start_input, prune_end_input)) {
      prune_range_start = -1;
      prune_range_end = -1;
      update_viewer();
    }
  }

  ImGui::TextUnformatted("Prune ranges");
  ImGui::BeginChild("prune_ranges", ImVec2(0.0f, 140.0f), true);
  if (requested_prune_ranges.empty()) {
    ImGui::TextDisabled("No ranges selected");
  }

  bool ranges_changed = false;
  for (int i = 0; i < requested_prune_ranges.size(); i++) {
    ImGui::PushID(i);
    int endpoints[2] = {requested_prune_ranges[i].first, requested_prune_ranges[i].last};
    ImGui::SetNextItemWidth(180.0f);
    if (ImGui::InputInt2("##prune_range", endpoints)) {
      if (is_prune_endpoint(endpoints[0]) && is_prune_endpoint(endpoints[1])) {
        if (endpoints[0] > endpoints[1]) {
          std::swap(endpoints[0], endpoints[1]);
        }
        requested_prune_ranges[i] = {endpoints[0], endpoints[1]};
        ranges_changed = true;
      } else {
        logger->warn("invalid prune range [{}, {}]", endpoints[0], endpoints[1]);
      }
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Remove")) {
      requested_prune_ranges.erase(requested_prune_ranges.begin() + i);
      ranges_changed = true;
      i--;
    }
    ImGui::PopID();
  }
  ImGui::EndChild();

  if (ranges_changed) {
    update_normalized_prune_ranges();
    update_viewer();
  }

  if (ImGui::Checkbox("Hide selected submaps", &hide_selected_submaps)) {
    update_viewer();
  }
  ImGui::Checkbox(prune_source_session ? "Show current map" : "Show incoming map", &show_other_session);
  ImGui::Text("Selected: %d submaps", count_selected_submaps(normalized_prune_ranges));

  ImGui::End();
  if (!show_submap_pruning_window) {
    prune_selection_mode = false;
  }
}

bool InteractiveViewer::is_prune_endpoint(int submap_id) const {
  if (submap_id < 0 || submap_id >= submaps.size() || submaps.empty()) {
    return false;
  }
  if (is_source_submap(submap_id) != prune_source_session) {
    return false;
  }
  return !is_unavailable_submap(submap_id);
}

bool InteractiveViewer::is_unavailable_submap(int submap_id) const {
  return submap_id >= 0 && submap_id < unavailable_submap_mask.size() && unavailable_submap_mask[submap_id];
}

bool InteractiveViewer::is_orphaned_submap(int submap_id) const {
  return submap_id >= 0 && submap_id < orphaned_submap_mask.size() && orphaned_submap_mask[submap_id];
}

bool InteractiveViewer::is_source_submap(int submap_id) const {
  return submap_id >= 0 && submap_id < submaps.size() && !submaps.empty() && submaps[submap_id]->session_id == submaps.back()->session_id;
}

bool InteractiveViewer::is_selected_for_pruning(int submap_id) const {
  const auto contains = [submap_id](const std::vector<SubmapRange>& ranges) {
    return std::any_of(ranges.begin(), ranges.end(), [submap_id](const SubmapRange& range) { return range.first <= submap_id && submap_id <= range.last; });
  };
  return contains(normalized_target_prune_ranges) || contains(normalized_source_prune_ranges);
}

int InteractiveViewer::count_selected_submaps(const std::vector<SubmapRange>& ranges) const {
  int count = 0;
  for (const int id : submap_ranges_to_ids(ranges)) {
    if (id < submaps.size() && !is_unavailable_submap(id)) {
      count++;
    }
  }
  return count;
}

bool InteractiveViewer::has_remaining_submaps(bool source) const {
  for (const auto& submap : submaps) {
    if (is_source_submap(submap->id) == source && !is_unavailable_submap(submap->id) && !is_selected_for_pruning(submap->id)) {
      return true;
    }
  }
  return false;
}

void InteractiveViewer::update_normalized_prune_ranges() {
  if (prune_source_session) {
    normalized_source_prune_ranges = normalize_submap_ranges(requested_source_prune_ranges);
  } else {
    normalized_target_prune_ranges = normalize_submap_ranges(requested_target_prune_ranges);
  }
}

bool InteractiveViewer::add_prune_range(int first, int last) {
  if (!is_prune_endpoint(first) || !is_prune_endpoint(last)) {
    logger->warn("prune range endpoints must be active submaps in the selected map");
    return false;
  }
  if (first > last) {
    std::swap(first, last);
  }

  auto& requested_prune_ranges = prune_source_session ? requested_source_prune_ranges : requested_target_prune_ranges;
  requested_prune_ranges.push_back({first, last});
  update_normalized_prune_ranges();
  return true;
}

void InteractiveViewer::set_prune_endpoint(int submap_id, bool start) {
  if (!is_prune_endpoint(submap_id)) {
    logger->warn("submap {} cannot be used as a prune range endpoint", submap_id);
    return;
  }

  if (start) {
    prune_range_start = submap_id;
    prune_start_input = submap_id;
  } else {
    prune_range_end = submap_id;
    prune_end_input = submap_id;
  }

  if (prune_range_start >= 0 && prune_range_end >= 0 && add_prune_range(prune_range_start, prune_range_end)) {
    prune_range_start = -1;
    prune_range_end = -1;
    update_viewer();
  }
}

void InteractiveViewer::begin_session_merge() {
  if (current_graph_edit_state != GraphEditState::SESSION_MERGE_PENDING || submaps.empty()) {
    return;
  }

  const int source_session_id = submaps.back()->session_id;
  std::vector<SubMap::ConstPtr> target_submaps;
  std::vector<SubMap::ConstPtr> source_submaps;
  // Preprocess exactly the submaps that will remain in the merge candidate.
  for (const auto& submap : submaps) {
    const bool unavailable = is_unavailable_submap(submap->id);
    if (submap->session_id == source_session_id) {
      if (!unavailable && !is_selected_for_pruning(submap->id)) {
        source_submaps.push_back(submap);
      }
    } else if (!unavailable && !is_selected_for_pruning(submap->id)) {
      target_submaps.push_back(submap);
    }
  }

  if (target_submaps.empty() || source_submaps.empty()) {
    logger->warn("session merge requires active target and source submaps");
    return;
  }

  pending_merge_options = std::make_unique<SessionMergeOptions>();
  pending_merge_options->prune_ranges = union_submap_ranges(normalized_target_prune_ranges, normalized_source_prune_ranges);

  show_submap_pruning_window = false;
  prune_selection_mode = false;
  session_merge_in_progress = true;
  logger->info("aligning sessions with {} target and {} source submaps", target_submaps.size(), source_submaps.size());
  manual_loop_close_modal->set_submaps(target_submaps, source_submaps);
}

void InteractiveViewer::reset_graph_edit_ui() {
  requested_target_prune_ranges.clear();
  normalized_target_prune_ranges.clear();
  requested_source_prune_ranges.clear();
  normalized_source_prune_ranges.clear();
  prune_range_start = -1;
  prune_range_end = -1;
  prune_start_input = 0;
  prune_end_input = 0;
  show_submap_pruning_window = false;
  prune_selection_mode = false;
  hide_selected_submaps = false;
  prune_source_session = false;
  show_other_session = false;
  session_merge_in_progress = false;
  pending_merge_options.reset();
  orphaned_submap_mask.clear();
  candidate_orphaned_subgraph_count = 0;
  candidate_diagnostic.clear();
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
      ImGui::TextUnformatted(("Submap ID : " + std::to_string(frame_id)).c_str());
      if (prune_selection_mode) {
        const bool valid_endpoint = is_prune_endpoint(frame_id);
        if (ImGui::MenuItem("Set prune range start", nullptr, false, valid_endpoint)) {
          set_prune_endpoint(frame_id, true);
        }
        if (ImGui::MenuItem("Set prune range end", nullptr, false, valid_endpoint)) {
          set_prune_endpoint(frame_id, false);
        }
      } else {
        const bool accepts_graph_factors = current_graph_edit_state.load() != GraphEditState::SESSION_MERGE_PENDING && !needs_session_merge.load();
        const bool active = accepts_graph_factors && !is_unavailable_submap(frame_id);
        if (ImGui::MenuItem("Loop begin", nullptr, manual_loop_close_modal->is_target_set(), active)) {
          manual_loop_close_modal->set_target(X(frame_id), submaps[frame_id]->frame, submap_poses[frame_id]);
        }
        if (ImGui::MenuItem("Loop end", nullptr, manual_loop_close_modal->is_source_set(), active)) {
          manual_loop_close_modal->set_source(X(frame_id), submaps[frame_id]->frame, submap_poses[frame_id]);
        }
      }
    }

    if (type == PickType::POINTS) {
      const bool accepts_graph_factors = current_graph_edit_state.load() != GraphEditState::SESSION_MERGE_PENDING && !needs_session_merge.load();
      if (ImGui::MenuItem("Bundle adjustment (Plane)", nullptr, false, accepts_graph_factors)) {
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

  auto manual_loop_close_factor = manual_loop_close_modal->run();
  const bool alignment_cancelled = manual_loop_close_modal->consume_cancelled();
  if (alignment_cancelled && session_merge_in_progress) {
    pending_merge_options.reset();
    session_merge_in_progress = false;
    show_submap_pruning_window = false;
    prune_selection_mode = false;
    logger->info("session merge alignment cancelled");
  }

  if (manual_loop_close_factor && pending_merge_options) {
    pending_merge_options->merge_factor = manual_loop_close_factor;

    GlobalMappingCallbacks::request_to_merge_sessions(*pending_merge_options);
    pending_merge_options.reset();
  } else {
    factors.push_back(manual_loop_close_factor);
  }
  factors.push_back(bundle_adjustment_modal->run());

  factors.erase(std::remove(factors.begin(), factors.end(), nullptr), factors.end());

  if (factors.size()) {
    gtsam::NonlinearFactorGraph graph;
    graph.add(factors);
    GlobalMappingCallbacks::request_to_add_graph_factors(graph);
  }
}

/**
 * @brief Update viewer
 */
void InteractiveViewer::update_viewer() {
  auto viewer = guik::LightViewer::instance();

  Eigen::Vector2f auto_z_range(0.0f, 0.0f);
  for (int i = 0; i < submaps.size(); i++) {
    const auto& submap = submaps[i];
    if (is_unavailable_submap(submap->id)) {
      viewer->remove_drawable("submap_" + std::to_string(submap->id));
      viewer->remove_drawable("coord_" + std::to_string(submap->id));
      viewer->remove_drawable("sphere_" + std::to_string(submap->id));
      continue;
    }

    const Eigen::Affine3f submap_pose = submap_poses[i].cast<float>();
    const Eigen::Vector4f session_color = glk::colormap_categoricalf(glk::COLORMAP::TURBO, submap->session_id, 6);
    const bool orphaned = is_orphaned_submap(submap->id);
    const bool selected_for_pruning = is_selected_for_pruning(submap->id);
    const Eigen::Vector4f point_color = orphaned               ? Eigen::Vector4f(1.0f, 0.0f, 0.0f, points_alpha)
                                        : selected_for_pruning ? Eigen::Vector4f(1.0f, 0.6f, 0.0f, points_alpha)
                                                               : session_color;

    auto_z_range[0] = std::min(auto_z_range[0], submap_pose.translation().z());
    auto_z_range[1] = std::max(auto_z_range[1], submap_pose.translation().z());

    auto drawable = viewer->find_drawable("submap_" + std::to_string(submap->id));
    if (drawable.first) {
      drawable.first->add("model_matrix", submap_pose.matrix());
      drawable.first->set_color(point_color).set_alpha(points_alpha);

      if (orphaned || selected_for_pruning) {
        drawable.first->set_color_mode(guik::ColorMode::FLAT_COLOR);
      } else {
        switch (color_mode) {
          case 0:
            drawable.first->set_color_mode(guik::ColorMode::RAINBOW);
            break;
          case 1:
            drawable.first->set_color_mode(guik::ColorMode::VERTEX_COLOR);
            break;
          case 2:
            drawable.first->set_color_mode(guik::ColorMode::FLAT_COLOR);
            break;
        }
      }
    } else {
      const Eigen::Vector4i info(static_cast<int>(PickType::POINTS), 0, 0, submap->id);
      auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(submap->frame->points, submap->frame->size());

      if (submap->frame->has_intensities()) {
        cloud_buffer->add_intensity(
          glk::COLORMAP::TURBO,
          submap->frame->intensities,
          submap->frame->size(),
          1.0 / *std::max_element(submap->frame->intensities, submap->frame->intensities + submap->frame->size()));
      }

      auto shader_setting = guik::Rainbow(submap_pose).add("info_values", info).set_color(point_color).set_alpha(points_alpha);
      if (orphaned || selected_for_pruning) {
        shader_setting.set_color_mode(guik::ColorMode::FLAT_COLOR);
      }

      if (enable_partial_rendering) {
        cloud_buffer->enable_partial_rendering(partial_rendering_budget);
        shader_setting.add("dynamic_object", 0).make_transparent();
      }

      viewer->update_drawable("submap_" + std::to_string(submap->id), cloud_buffer, shader_setting);
    }

    const Eigen::Vector4i info(static_cast<int>(PickType::FRAME), 0, 0, submap->id);

    viewer->update_drawable(
      "coord_" + std::to_string(submap->id),
      glk::Primitives::coordinate_system(),
      guik::VertexColor(submap_pose * Eigen::UniformScaling<float>(coord_scale)).add("info_values", info));

    viewer->update_drawable(
      "sphere_" + std::to_string(submap->id),
      glk::Primitives::sphere(),
      guik::FlatColor(
        orphaned               ? Eigen::Vector4f(1.0f, 0.0f, 1.0f, 0.8f)
        : selected_for_pruning ? Eigen::Vector4f(1.0f, 0.6f, 0.0f, 0.7f)
                               : Eigen::Vector4f(1.0f, 0.0f, 0.0f, 0.5f),
        submap_pose * Eigen::UniformScaling<float>(sphere_scale))
        .add("info_values", info)
        .make_transparent());
  }

  viewer->shader_setting().add<Eigen::Vector2f>("z_range", z_range + auto_z_range);

  std::vector<Eigen::Vector3f> factor_lines;
  std::vector<Eigen::Vector4f> factor_colors;
  factor_lines.reserve(global_factors.size() * 2);
  factor_colors.reserve(global_factors.size() * 2);

  const auto get_position = [this](const gtsam::Key key) -> Eigen::Vector3d {
    gtsam::Symbol symbol(key);
    switch (symbol.chr()) {
      case 'x':
        return submap_poses[symbol.index()].translation();
      case 'e': {
        const int right = symbol.index() % 2;
        const int submap_id = (symbol.index() - right) / 2;

        const auto& submap = submaps[submap_id];
        const auto& T_origin_endpoint = right ? submap->T_origin_endpoint_R : submap->T_origin_endpoint_L;
        const Eigen::Isometry3d T_world_endpoint = submap_poses[submap_id] * T_origin_endpoint;

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
        color = Eigen::Vector4f(0.0f, 1.0f, 0.0f, factors_alpha);
        break;
      case FactorType::BETWEEN:
        color = Eigen::Vector4f(0.0f, 0.0f, 1.0f, factors_alpha);
        break;
      case FactorType::IMU:
        color = Eigen::Vector4f(1.0f, 0.0f, 0.0f, factors_alpha);
        break;
    }

    factor_colors.push_back(color);
    factor_colors.push_back(color);
  }

  viewer->update_drawable("factors", std::make_shared<glk::ThinLines>(factor_lines, factor_colors), guik::VertexColor().set_alpha(factors_alpha));

  viewer->remove_drawable(std::regex("traj.*"));
  std::vector<Eigen::Vector3f> traj;
  int trajectory_segment = 0;
  int previous_submap_id = -1;
  int previous_session_id = -1;
  const auto draw_trajectory_segment = [&] {
    if (traj.empty()) {
      return;
    }
    auto traj_line = std::make_shared<glk::ThinLines>(traj, true);
    traj_line->set_line_width(2.0f);
    viewer->update_drawable("traj_" + std::to_string(trajectory_segment++), traj_line, guik::FlatGreen());
    traj.clear();
  };

  for (int i = 0; i < submaps.size(); i++) {
    const auto& submap = submaps[i];
    if (is_unavailable_submap(submap->id)) {
      draw_trajectory_segment();
      previous_submap_id = -1;
      previous_session_id = -1;
      continue;
    }

    if (previous_submap_id >= 0 && (submap->session_id != previous_session_id || submap->id != previous_submap_id + 1)) {
      draw_trajectory_segment();
    }

    const Eigen::Isometry3d T_world_endpoint_L = submap_poses[i] * submap->T_origin_endpoint_L;
    const Eigen::Isometry3d T_odom_imu0 = submap->frames.front()->T_world_imu;
    for (const auto& frame : submap->frames) {
      const Eigen::Isometry3d T_world_imu = T_world_endpoint_L * T_odom_imu0.inverse() * frame->T_world_imu;
      traj.emplace_back(T_world_imu.translation().cast<float>());
    }

    previous_submap_id = submap->id;
    previous_session_id = submap->session_id;
  }
  draw_trajectory_segment();
}

void InteractiveViewer::odometry_on_new_frame(const EstimationFrame::ConstPtr& new_frame) {
  invoke([this, new_frame] {
    auto viewer = guik::viewer();
    auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(new_frame->frame->points, new_frame->frame->size());

    trajectory->add_odom(new_frame->stamp, new_frame->T_world_sensor(), 1);
    const Eigen::Isometry3d pose = trajectory->odom2world(new_frame->T_world_sensor());
    viewer->update_drawable("current", cloud_buffer, guik::FlatOrange(pose).set_point_scale(2.0f));
  });
}

/**
 * @brief New submap insertion callback
 */
void InteractiveViewer::globalmap_on_insert_submap(const SubMap::ConstPtr& submap) {
  std::shared_ptr<Eigen::Isometry3d> pose(new Eigen::Isometry3d(submap->T_world_origin));
  invoke([this, submap, pose] {
    if (submaps.size() && submaps.back()->session_id != submap->session_id) {
      needs_session_merge = true;
    }

    trajectory->update_anchor(submap->frames[submap->frames.size() / 2]->stamp, submap->T_world_origin);

    submap_poses.push_back(*pose);
    submaps.push_back(submap);
    update_viewer();
  });
}

/**
 * @brief Submap pose update callback
 */
void InteractiveViewer::globalmap_on_update_submaps(const std::vector<SubMap::Ptr>& updated_submaps) {
  std::vector<Eigen::Isometry3d> poses(updated_submaps.size());
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
void InteractiveViewer::globalmap_on_smoother_update(gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) {
  const auto inserted_factors = extract_display_factors(new_factors);
  invoke([this, inserted_factors] { global_factors.insert(global_factors.end(), inserted_factors.begin(), inserted_factors.end()); });
}

/**
 * @brief Smoother update result callback
 */
void InteractiveViewer::globalmap_on_smoother_update_result(gtsam_points::ISAM2Ext& isam2, const gtsam_points::ISAM2ResultExt& result) {
  const std::string text = result.to_string();
  logger->info("--- smoother_updated ---\n{}", text);
}

void InteractiveViewer::globalmap_on_graph_edit_state_changed(GraphEditState state, const std::vector<uint8_t>& pruned_mask) {
  const GraphEditState previous = current_graph_edit_state.exchange(state);
  needs_session_merge = state == GraphEditState::SESSION_MERGE_PENDING;

  invoke([this, state, previous, pruned_mask] {
    unavailable_submap_mask = pruned_mask;
    if (state == GraphEditState::IDLE && previous != GraphEditState::IDLE) {
      reset_graph_edit_ui();
    } else if (state == GraphEditState::SESSION_MERGE_PENDING || state == GraphEditState::CANDIDATE_EDITING) {
      session_merge_in_progress = false;
    }
    update_viewer();
  });
}

void InteractiveViewer::globalmap_on_candidate_graph_updated(const CandidateGraph& candidate) {
  std::vector<std::pair<int, Eigen::Isometry3d>> poses;
  for (const auto& value : candidate.values) {
    const gtsam::Symbol symbol(value.key);
    if (symbol.chr() == 'x') {
      poses.emplace_back(symbol.index(), Eigen::Isometry3d(value.value.cast<gtsam::Pose3>().matrix()));
    }
  }

  std::vector<int> orphaned_submap_ids;
  for (const auto key : candidate.connectivity.unreachable_pose_keys) {
    orphaned_submap_ids.push_back(gtsam::Symbol(key).index());
  }

  const auto factors = extract_display_factors(candidate.factors);
  const std::size_t orphaned_subgraph_count = candidate.connectivity.orphaned_subgraph_count();
  const std::string diagnostic = candidate.diagnostic;
  invoke([this, poses, orphaned_submap_ids, factors, orphaned_subgraph_count, diagnostic] {
    for (const auto& [id, pose] : poses) {
      submap_poses[id] = pose;
    }
    orphaned_submap_mask.assign(submaps.size(), 0);
    for (const int id : orphaned_submap_ids) {
      orphaned_submap_mask[id] = 1;
    }
    global_factors = factors;
    candidate_orphaned_subgraph_count = orphaned_subgraph_count;
    candidate_diagnostic = diagnostic;
    update_viewer();
  });
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
  unavailable_submap_mask.clear();
  current_graph_edit_state = GraphEditState::IDLE;
  needs_session_merge = false;
  reset_graph_edit_ui();

  guik::LightViewer::instance()->clear_drawables();
}

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::InteractiveViewer();
}
