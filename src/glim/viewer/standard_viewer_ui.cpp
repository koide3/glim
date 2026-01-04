#include <glim/viewer/standard_viewer.hpp>

#include <spdlog/spdlog.h>

#include <gtsam_points/config.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim/odometry/callbacks.hpp>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/trajectory_manager.hpp>

#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/spdlog_sink.hpp>
#include <guik/viewer/light_viewer.hpp>

#include <glim/viewer/standard_viewer_mem.hpp>

namespace glim {

bool StandardViewer::drawable_filter(const std::string& name) {
  const auto starts_with = [](const std::string& name, const std::string& pattern) {
    if (name.size() < pattern.size()) {
      return false;
    }

    return std::equal(pattern.begin(), pattern.end(), name.begin());
  };

  if (!show_current_coord && name == "current_coord") {
    return false;
  }

  if (!show_current_points && name == "current_frame") {
    return false;
  }

  if (!show_odometry_scans && starts_with(name, "frame_")) {
    return false;
  }

  if (!show_odometry_keyframes && starts_with(name, "odometry_keyframe_")) {
    return false;
  }

  if (!show_odometry_factors && starts_with(name, "odometry_factors")) {
    return false;
  }

  if (!show_submaps && starts_with(name, "submap_")) {
    return false;
  }

  if (!show_factors && starts_with(name, "factors")) {
    return false;
  }

  return true;
}

void StandardViewer::drawable_selection() {
  auto viewer = guik::LightViewer::instance();

  ImGui::SetWindowPos("images", {1800, 60}, ImGuiCond_FirstUseEver);
  ImGui::SetWindowPos("logging", {1800, 950}, ImGuiCond_FirstUseEver);

  ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.5));
  ImGui::Begin("selection", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::PopStyleColor();

  if (ImGui::Checkbox("track", &track)) {
    if (track) {
      guik::LightViewer::instance()->reset_center();
    }
  }
  ImGui::SameLine();
  bool show_current = show_current_coord || show_current_points;
  if (ImGui::Checkbox("current", &show_current)) {
    show_current_coord = show_current_points = show_current;
  }
  ImGui::SameLine();
  ImGui::Checkbox("coord", &show_current_coord);
  ImGui::SameLine();
  ImGui::Checkbox("points", &show_current_points);

  ImGui::Separator();
  bool show_odometry = show_odometry_scans || show_odometry_keyframes || show_odometry_factors;
  if (ImGui::Checkbox("odometry", &show_odometry)) {
    show_odometry_scans = show_odometry_keyframes = show_odometry;
    show_odometry_factors &= show_odometry;
  }

  ImGui::SameLine();
  if (ImGui::Button("Status")) {
    show_odometry_status = true;
  }

  ImGui::Checkbox("scans##odom", &show_odometry_scans);
  ImGui::SameLine();
  ImGui::Checkbox("keyframes##odom", &show_odometry_keyframes);
  ImGui::SameLine();
  ImGui::Checkbox("factors##odom", &show_odometry_factors);

  ImGui::Separator();
  bool show_mapping = show_submaps || show_factors;
  if (ImGui::Checkbox("mapping", &show_mapping)) {
    show_submaps = show_factors = show_mapping;
  }

  ImGui::SameLine();
  if (ImGui::Button("Tools")) {
    show_mapping_tools = true;
  }

  ImGui::SameLine();
  if (ImGui::Button("Mem stats")) {
    show_memory_stats = true;
  }

  ImGui::SameLine();
  if (ImGui::Button("Log")) {
    viewer->register_ui_callback("logging", guik::create_logger_ui(glim::get_ringbuffer_sink(), 0.5));
  }

  ImGui::Checkbox("submaps", &show_submaps);
  ImGui::SameLine();
  ImGui::Checkbox("factors", &show_factors);

  ImGui::Separator();

  std::vector<const char*> odom_color_modes = {"FLAT", "INTENSITY", "NORMAL"};
  ImGui::SetNextItemWidth(92);
  ImGui::Combo("odom_color_mode", &odom_color_mode, odom_color_modes.data(), odom_color_modes.size());

  std::vector<const char*> submap_color_modes = {"RAINBOW", "INTENSITY", "COLOR"};
  ImGui::SetNextItemWidth(92);
  if (ImGui::Combo("submap_color_mode", &submap_color_mode, submap_color_modes.data(), submap_color_modes.size())) {
    for (int i = 0;; i++) {
      const auto found = viewer->find_drawable("submap_" + std::to_string(i));
      if (!found.first) {
        break;
      }

      switch (submap_color_mode) {
        case 0:
          found.first->set_color_mode(guik::ColorMode::RAINBOW);
          break;
        case 1:
          found.first->set_color_mode(guik::ColorMode::VERTEX_COLORMAP);
          break;
        case 2:
          found.first->set_color_mode(guik::ColorMode::VERTEX_COLOR);
          break;
      }
    }
  }

  if (submap_color_mode == 0) {
    std::vector<const char*> z_range_modes = {"AUTO", "LOCAL", "MANUAL"};
    bool update_z_range = false;

    ImGui::SetNextItemWidth(150);
    update_z_range |= ImGui::Combo("z_range_mode", &z_range_mode, z_range_modes.data(), z_range_modes.size());
    update_z_range |= ImGui::DragFloatRange2("z_range", &z_range[0], &z_range[1], 0.1f, -10000.0f, 10000.0f);
    if (update_z_range) {
      Eigen::Vector2f z = z_range;
      if (z_range_mode == 0) {
        z += auto_z_range;
      } else if (z_range_mode == 1) {
        z += Eigen::Vector2f::Constant(last_submap_z);
      }
      viewer->shader_setting().add<Eigen::Vector2f>("z_range", z);
    }
  }

  if (odom_color_mode == 1 || submap_color_mode == 1) {
    ImGui::Checkbox("auto_intensity_range", &auto_intensity_range);
    if (auto_intensity_range) {
      intensity_range[0] = intensity_dist.min();
      intensity_range[1] = intensity_dist.max();
    }

    ImGui::SetNextItemWidth(150);
    ImGui::DragFloatRange2("intensity_range", &intensity_range[0], &intensity_range[1], 0.1f, -65536.0f, 65536.0f);
    viewer->shader_setting().add<Eigen::Vector2f>("cmap_range", Eigen::Vector2f(intensity_range[0], intensity_range[1]));
  }

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

  ImGui::End();

  if (show_odometry_status) {
    ImGui::Begin("odometry status", &show_odometry_status, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::Text("frame ID:%d", last_id);
    ImGui::Text("points:%d", last_num_points);
    ImGui::Text("median dist:%.3f", last_median_distance);

    std::stringstream sst;
    if (last_voxel_resolutions.empty()) {
      sst << "voxel_resolution: N/A";
    } else {
      sst << "voxel_resolution: ";
      for (double r : last_voxel_resolutions) {
        sst << fmt::format("{:.3f}", r) << " ";
      }
    }
    const std::string text = sst.str();
    ImGui::Text("%s", text.c_str());

    ImGui::Text("stamp:%.3f ~ %.3f", last_point_stamps.first, last_point_stamps.second);
    ImGui::Text("vel:%.3f %.3f %.3f", last_imu_vel[0], last_imu_vel[1], last_imu_vel[2]);
    ImGui::Text("bias:%.3f %.3f %.3f %.3f %.3f %.3f", last_imu_bias[0], last_imu_bias[1], last_imu_bias[2], last_imu_bias[3], last_imu_bias[4], last_imu_bias[5]);
    ImGui::End();
  }

  if (show_mapping_tools) {
    ImGui::Begin("mapping tools", &show_mapping_tools, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::DragFloat("Min overlap", &min_overlap, 0.01f, 0.01f, 1.0f);
    if (ImGui::Button("Find overlapping submaps")) {
      logger->info("finding overlapping submaps...");
      GlobalMappingCallbacks::request_to_find_overlapping_submaps(min_overlap);
    }

    if (ImGui::Button("Optimize")) {
      logger->info("optimizing...");
      GlobalMappingCallbacks::request_to_optimize();
    }
    ImGui::End();
  }

  if (show_memory_stats) {
    ImGui::Begin("memory stats", &show_memory_stats, ImGuiWindowFlags_AlwaysAutoResize);

    size_t points_cpu = 0;
    size_t points_gpu = 0;
    size_t voxelmap_cpu = 0;
    size_t voxelmap_gpu = 0;
    size_t odom_cpu = 0;
    size_t odom_gpu = 0;

    for (const auto& m : submap_memstats) {
      points_cpu += m.frame_cpu_bytes;
      points_gpu += m.frame_gpu_bytes;
      voxelmap_cpu += m.voxelmap_cpu_bytes;
      voxelmap_gpu += m.voxelmap_gpu_bytes;
      odom_cpu += m.odom_cpu_bytes;
      odom_gpu += m.odom_gpu_bytes;
    }

    size_t factors_cpu = 0;
    size_t factors_gpu = 0;

    for (const auto& m : global_factor_memstats) {
      factors_cpu += m.cpu_bytes;
      factors_gpu += m.gpu_bytes;
    }

    constexpr double mb = 1.0 / (1024.0 * 1024.0);
    const size_t total_cpu = points_cpu + voxelmap_cpu + odom_cpu + factors_cpu;
    const size_t total_gpu = points_gpu + voxelmap_gpu + odom_gpu + factors_gpu + total_gl_bytes;
    const double total_cpu_mb = total_cpu * mb;
    const double total_gpu_mb = total_gpu * mb;

    ImGui::Text("Global mapping memory usage");
    if (ImGui::BeginTable("Global mapping memory usage", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
      const auto show_item = [=](const char* name, size_t cpu, size_t gpu) {
        const double cpu_mb = cpu * mb;
        const double gpu_mb = gpu * mb;

        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::Text("%s", name);
        ImGui::TableNextColumn();
        ImGui::Text("%.2f MB / %.1f %%", cpu_mb, cpu_mb / total_cpu_mb * 100.0);
        ImGui::TableNextColumn();
        ImGui::Text("%.2f MB / %.1f %%", gpu * mb, gpu_mb / total_gpu_mb * 100.0);
      };

      ImGui::TableSetupColumn("Item");
      ImGui::TableSetupColumn("CPU");
      ImGui::TableSetupColumn("GPU");
      ImGui::TableHeadersRow();

      show_item("Total", total_cpu, total_gpu);
      show_item("Points", points_cpu, points_gpu);
      show_item("Voxelmap", voxelmap_cpu, voxelmap_gpu);
      show_item("Odom frames", odom_cpu, odom_gpu);
      show_item("Factors", factors_cpu, factors_gpu);
      show_item("OpenGL", 0, total_gl_bytes);

      ImGui::EndTable();
    }

    ImGui::End();
  }
}
}  // namespace glim