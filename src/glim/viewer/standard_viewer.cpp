#include <glim/viewer/standard_viewer.hpp>

#include <spdlog/spdlog.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_points/config.hpp>
#include <gtsam_points/util/gtsam_migration.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/factors/integrated_matching_cost_factor.hpp>
#include <gtsam_points/optimizers/isam2_result_ext.hpp>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_optimization_status.hpp>

#ifdef GTSAM_POINTS_USE_CUDA
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#endif

#include <glim/odometry/callbacks.hpp>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/trajectory_manager.hpp>

#include <glk/colormap.hpp>
#include <glk/texture.hpp>
#ifdef GLIM_USE_OPENCV
#include <glk/texture_opencv.hpp>
#endif
#include <glk/thin_lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/indexed_pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/spdlog_sink.hpp>
#include <guik/viewer/light_viewer.hpp>

#include <glim/viewer/standard_viewer_mem.hpp>

namespace glim {

StandardViewer::StandardViewer() : logger(create_module_logger("viewer")) {
  glim::Config config(glim::GlobalConfig::get_config_path("config_viewer"));

  viewer_started = false;
  kill_switch = false;
  request_to_terminate = false;

  track = true;
  show_current_coord = true;
  show_current_points = true;
  odom_color_mode = 0;
  submap_color_mode = 0;

  show_odometry_scans = true;
  show_odometry_keyframes = true;
  show_odometry_factors = false;
  show_submaps = true;
  show_factors = true;

  show_odometry_status = false;
  last_id = last_num_points = 0;
  last_point_stamps = std::make_pair(0.0, 0.0);
  last_imu_vel.setZero();
  last_imu_bias.setZero();
  last_median_distance = 0.0;

  z_range_mode = 0;
  z_range = config.param("standard_viewer", "default_z_range", Eigen::Vector2d(-2.0, 4.0)).cast<float>();
  auto_z_range << 0.0f, 0.0f;
  last_submap_z = 0.0;

  show_mapping_tools = false;
  min_overlap = 0.2f;

  show_memory_stats = false;
  submap_memstats_count = 0;
  global_factor_stats_count = 0;
  total_gl_bytes = 0;

  points_alpha = config.param("standard_viewer", "points_alpha", 1.0);
  factors_alpha = config.param("standard_viewer", "factors_alpha", 1.0);

  auto_intensity_range = true;
  intensity_range = Eigen::Vector2f(0.0f, 1.0f);
  intensity_dist.add(0.0);
  intensity_dist.add(1.0);

  point_size = config.param("standard_viewer", "point_size", 0.025);
  point_size_metric = config.param("standard_viewer", "point_size_metric", true);
  point_shape_circle = config.param("standard_viewer", "point_shape_circle", true);

  trajectory.reset(new TrajectoryManager);

  enable_partial_rendering = config.param("standard_viewer", "enable_partial_rendering", false);
  partial_rendering_budget = config.param("standard_viewer", "partial_rendering_budget", 1024);

  set_callbacks();
  thread = std::thread([this] { viewer_loop(); });

  const auto t1 = std::chrono::high_resolution_clock::now();
  while (!viewer_started && std::chrono::high_resolution_clock::now() - t1 < std::chrono::seconds(1)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (!viewer_started) {
    logger->critical("Timeout waiting for viewer to start");
  }
}

StandardViewer::~StandardViewer() {
  kill_switch = true;
  if (thread.joinable()) {
    thread.join();
  }
}

bool StandardViewer::ok() const {
  return !request_to_terminate;
}

void StandardViewer::invoke(const std::function<void()>& task) {
  if (kill_switch) {
    return;
  }
  std::lock_guard<std::mutex> lock(invoke_queue_mutex);
  invoke_queue.push_back(task);
}

Eigen::Isometry3f StandardViewer::resolve_pose(const EstimationFrame::ConstPtr& frame) {
  switch (frame->frame_id) {
    case FrameID::WORLD:
      return Eigen::Isometry3f::Identity();

    default:
      return trajectory->odom2world(frame->T_world_sensor()).cast<float>();
  }
}

void StandardViewer::viewer_loop() {
  glim::Config config(glim::GlobalConfig::get_config_path("config_viewer"));

  auto viewer = guik::LightViewer::instance(Eigen::Vector2i(config.param("standard_viewer", "viewer_width", 2560), config.param("standard_viewer", "viewer_height", 1440)));
  viewer_started = true;

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
    viewer->enable_partial_rendering(1e-1);
    viewer->shader_setting().add("dynamic_object", 1);
  }

  auto submap_viewer = viewer->sub_viewer("submap");
  submap_viewer->set_pos(Eigen::Vector2i(100, 800));
  submap_viewer->set_draw_xy_grid(false);
  submap_viewer->use_topdown_camera_control(80.0);

  viewer->register_drawable_filter("selection", [this](const std::string& name) { return drawable_filter(name); });
  viewer->register_ui_callback("selection", [this] { drawable_selection(); });
  viewer->register_ui_callback("logging", guik::create_logger_ui(glim::get_ringbuffer_sink(), 0.5));

  while (!kill_switch) {
    if (!viewer->spin_once()) {
      request_to_terminate = true;
    }

    std::vector<std::function<void()>> tasks;
    {
      std::lock_guard<std::mutex> lock(invoke_queue_mutex);
      tasks.swap(invoke_queue);
    }

    for (const auto& task : tasks) {
      task();
    }
  }

  guik::LightViewer::destroy();
}

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::StandardViewer();
}
