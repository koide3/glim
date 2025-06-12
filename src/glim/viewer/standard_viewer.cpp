#include <glim/viewer/standard_viewer.hpp>

#include <spdlog/spdlog.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_points/config.hpp>
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
#include <imgui.h> // Required for ImGui calls
#include <portable-file-dialogs.h> // For Quit confirmation

namespace glim {

StandardViewer::StandardViewer() : logger(create_module_logger("viewer")) {
  glim::Config config(glim::GlobalConfig::get_config_path("config_viewer"));

  viewer_started = false;
  kill_switch = false;
  request_to_terminate = false;

  track = true;
  show_current_coord = true;
  show_current_points = true;
  current_color_mode = 0;

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

  points_alpha = config.param("standard_viewer", "points_alpha", 1.0);
  factors_alpha = config.param("standard_viewer", "factors_alpha", 1.0);

  point_size = config.param("standard_viewer", "point_size", 1.0);
  point_size_metric = config.param("standard_viewer", "point_size_metric", false);
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

void StandardViewer::set_callbacks() {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  /*** Odometry callbacks ***/

  // IMU state initialization update callback
  IMUStateInitializationCallbacks::on_updated.add([this](const PreprocessedFrame::ConstPtr& frame, const Eigen::Isometry3d& T_odom_lidar_) {
    std::shared_ptr<Eigen::Isometry3d> T_odom_lidar(new Eigen::Isometry3d(T_odom_lidar_));
    invoke([this, frame, T_odom_lidar] {
      auto viewer = guik::LightViewer::instance();
      auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(frame->points);
      viewer->update_drawable("initialization_frame_" + guik::anon(), cloud_buffer, guik::FlatBlue(*T_odom_lidar));
      viewer->update_drawable("initialization_frame_current", cloud_buffer, guik::FlatOrange(*T_odom_lidar).set_point_scale(2.0f));
    });
  });

  // IMU state initialization termination callback
  IMUStateInitializationCallbacks::on_finished.add([this](const EstimationFrame::ConstPtr& frame) {
    invoke([this] {
      auto viewer = guik::LightViewer::instance();
      viewer->remove_drawable(std::regex("initialization.*"));
    });
  });

#ifdef GLIM_USE_OPENCV
  // New image callback
  OdometryEstimationCallbacks::on_insert_image.add([this](const double stamp, const cv::Mat& image) {
    invoke([this, image] {
      auto viewer = guik::LightViewer::instance();
      const auto texture = glk::create_texture(image);
      viewer->update_image("image", texture);
    });
  });
#endif

  // New frame callback
  OdometryEstimationCallbacks::on_new_frame.add([this](const EstimationFrame::ConstPtr& new_frame) {
    invoke([this, new_frame] {
      auto viewer = guik::LightViewer::instance();
      auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(new_frame->frame->points, new_frame->frame->size());

      last_id = new_frame->id;
      last_num_points = new_frame->frame->size();
      if (new_frame->raw_frame && new_frame->raw_frame->size()) {
        last_point_stamps.first = new_frame->raw_frame->times.front();
        last_point_stamps.second = new_frame->raw_frame->times.back();
      }
      last_imu_vel = new_frame->v_world_imu;
      last_imu_bias = new_frame->imu_bias;
      last_median_distance = gtsam_points::median_distance(new_frame->frame, 256);
      last_voxel_resolutions.clear();
      for (const auto& voxelmap : new_frame->voxelmaps) {
        if (voxelmap) {
          last_voxel_resolutions.emplace_back(voxelmap->voxel_resolution());
        }
      }

      trajectory->add_odom(new_frame->stamp, new_frame->T_world_sensor(), 1);
      const Eigen::Isometry3f pose = resolve_pose(new_frame);

      if (track) {
        viewer->lookat(pose.translation());
      }

      guik::ShaderSetting shader_setting = guik::FlatColor(1.0f, 0.5f, 0.0f, 1.0f, pose);
      guik::ShaderSetting shader_setting_rainbow = guik::Rainbow(pose);

      switch (current_color_mode) {
        case 0:  // FLAT
          break;
        case 1:  // INTENSITY
          if (new_frame->raw_frame && !new_frame->raw_frame->intensities.empty()) {
            const double max_intensity = *std::max_element(new_frame->raw_frame->intensities.begin(), new_frame->raw_frame->intensities.end());
            cloud_buffer->add_intensity(glk::COLORMAP::TURBO, new_frame->raw_frame->intensities, 1.0 / max_intensity);
          } else if (new_frame->frame->intensities) {
            std::vector<float> intensities(new_frame->frame->intensities, new_frame->frame->intensities + new_frame->frame->size());
            const float max_intensity = *std::max_element(intensities.begin(), intensities.end());
            cloud_buffer->add_intensity(glk::COLORMAP::TURBO, intensities, 1.0f / max_intensity);
          }
          shader_setting.add("color_mode", guik::ColorMode::VERTEX_COLOR);
          shader_setting_rainbow.add("color_mode", guik::ColorMode::VERTEX_COLOR);
          break;
        case 2:  // NORMAL
          if (new_frame->frame->normals) {
            std::vector<Eigen::Vector4d> normals(new_frame->frame->normals, new_frame->frame->normals + new_frame->frame->size());
            for (auto& normal : normals) {
              normal = normal.array().abs();
              normal[3] = 1.0;
            }
            cloud_buffer->add_color(normals);
          }

          shader_setting.add("color_mode", guik::ColorMode::VERTEX_COLOR);
          shader_setting_rainbow.add("color_mode", guik::ColorMode::VERTEX_COLOR);
          break;
      }

      viewer->update_drawable("current_frame", cloud_buffer, shader_setting.add("point_scale", 2.0f));
      viewer->update_drawable("current_coord", glk::Primitives::coordinate_system(), guik::VertexColor(pose * Eigen::UniformScaling<float>(1.5f)));
      viewer->update_drawable("frame_" + std::to_string(new_frame->id), cloud_buffer, shader_setting_rainbow);
    });
  });

  // Update frames callback
  OdometryEstimationCallbacks::on_update_frames.add([this](const std::vector<EstimationFrame::ConstPtr>& frames) {
    invoke([this, frames] {
      auto viewer = guik::LightViewer::instance();
      for (const auto& frame : frames) {
        const Eigen::Isometry3f pose = resolve_pose(frame);
        odometry_poses[frame->id] = pose;

        viewer->update_drawable(
          "frame_coord_" + std::to_string(frame->id),
          glk::Primitives::coordinate_system(),
          guik::VertexColor(resolve_pose(frame) * Eigen::UniformScaling<float>(0.5f)));
        auto drawable = viewer->find_drawable("frame_" + std::to_string(frame->id));
        if (drawable.first) {
          drawable.first->add<Eigen::Matrix4f>("model_matrix", pose.matrix());
        }
      }
    });
  });

  // Update keyframes callback
  OdometryEstimationCallbacks::on_update_keyframes.add([this](const std::vector<EstimationFrame::ConstPtr>& keyframes) {
    invoke([this, keyframes] {
      auto viewer = guik::LightViewer::instance();

      for (const auto& keyframe : keyframes) {
        const Eigen::Isometry3f pose = resolve_pose(keyframe);
        odometry_poses[keyframe->id] = pose;

        const std::string name = "odometry_keyframe_" + std::to_string(keyframe->id);
        auto drawable = viewer->find_drawable(name);
        if (drawable.first == nullptr) {
          auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(keyframe->frame->points, keyframe->frame->size());
          viewer->update_drawable(name, cloud_buffer, guik::Rainbow(pose));
        } else {
          drawable.first->add("model_matrix", pose.matrix());
        }

        viewer->update_drawable("odometry_keyframe_coord_" + std::to_string(keyframe->id), glk::Primitives::coordinate_system(), guik::VertexColor(pose));
      }
    });
  });

  OdometryEstimationCallbacks::on_smoother_update.add([this](
                                                        gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
                                                        gtsam::NonlinearFactorGraph& new_factors,
                                                        gtsam::Values& new_values,
                                                        std::map<std::uint64_t, double>& new_stamps) {
    //
    std::vector<std::pair<boost::weak_ptr<gtsam::NonlinearFactor>, FactorLineGetter>> new_factor_lines;
    new_factor_lines.reserve(new_factors.size());

    for (const auto& factor : new_factors) {
      if (!factor) {
        continue;
      }

      if (factor->keys().size() == 1) {
        const gtsam::Symbol symbol0(factor->keys()[0]);
        if (symbol0.chr() != 'x') {
          continue;
        }
        const int idx0 = symbol0.index();

        const auto matching_factor = boost::dynamic_pointer_cast<gtsam_points::IntegratedMatchingCostFactor>(factor);
        if (matching_factor) {
          const auto l = [this, idx0](const gtsam::NonlinearFactor* factor) -> std::optional<FactorLine> {
            const auto found0 = odometry_poses.find(idx0);
            if (found0 == odometry_poses.end()) {
              return std::nullopt;
            }

            const Eigen::Vector3d pt1 = static_cast<const gtsam_points::IntegratedMatchingCostFactor*>(factor)->get_fixed_target_pose().translation();
            return std::make_tuple(
              found0->second.translation(),
              pt1.cast<float>(),
              Eigen::Vector4f(0.0f, 1.0f, 0.0f, factors_alpha),
              Eigen::Vector4f(1.0f, 0.0f, 0.0f, factors_alpha));
          };

          new_factor_lines.emplace_back(factor, l);
          continue;
        }

#ifdef GTSAM_POINTS_USE_CUDA
        const auto gpu_factor = boost::dynamic_pointer_cast<gtsam_points::IntegratedVGICPFactorGPU>(factor);
        if (gpu_factor) {
          const auto l = [this, idx0](const gtsam::NonlinearFactor* factor) -> std::optional<FactorLine> {
            const auto found0 = odometry_poses.find(idx0);
            if (found0 == odometry_poses.end()) {
              return std::nullopt;
            }

            const Eigen::Vector3f pt1 = static_cast<const gtsam_points::IntegratedVGICPFactorGPU*>(factor)->get_fixed_target_pose().translation();
            return std::make_tuple(found0->second.translation(), pt1, Eigen::Vector4f(0.0f, 1.0f, 0.0f, factors_alpha), Eigen::Vector4f(1.0f, 0.0f, 0.0f, factors_alpha));
          };

          new_factor_lines.emplace_back(factor, l);
          continue;
        }
#endif
      } else if (factor->keys().size() == 2) {
        const gtsam::Symbol symbol0(factor->keys()[0]);
        const gtsam::Symbol symbol1(factor->keys()[1]);
        if (symbol0.chr() != 'x' || symbol1.chr() != 'x') {
          continue;
        }

        const int idx0 = symbol0.index();
        const int idx1 = symbol1.index();

        const auto l = [this, idx0, idx1](const gtsam::NonlinearFactor*) -> std::optional<FactorLine> {
          const auto found0 = odometry_poses.find(idx0);
          const auto found1 = odometry_poses.find(idx1);
          if (found0 == odometry_poses.end() || found1 == odometry_poses.end()) {
            return std::nullopt;
          }

          return std::make_tuple(
            found0->second.translation(),
            found1->second.translation(),
            Eigen::Vector4f(0.0f, 1.0f, 0.0f, factors_alpha),
            Eigen::Vector4f(0.0f, 1.0f, 0.0f, factors_alpha));
        };

        new_factor_lines.emplace_back(factor, l);
      }
    }

    invoke([this, new_factor_lines] {
      auto remove_loc = std::remove_if(odometry_factor_lines.begin(), odometry_factor_lines.end(), [](const auto& factor) { return factor.first.expired(); });
      odometry_factor_lines.erase(remove_loc, odometry_factor_lines.end());
      odometry_factor_lines.insert(odometry_factor_lines.end(), new_factor_lines.begin(), new_factor_lines.end());

      if (!show_odometry_factors) {
        return;
      }

      std::vector<Eigen::Vector3f> line_vertices;
      std::vector<Eigen::Vector4f> line_colors;

      for (const auto& factor_line : odometry_factor_lines) {
        const auto factor = factor_line.first.lock();
        if (!factor) {
          continue;
        }

        const auto line = factor_line.second(factor.get());
        if (!line) {
          continue;
        }
        line_vertices.push_back(std::get<0>(*line));
        line_vertices.push_back(std::get<1>(*line));
        line_colors.push_back(std::get<2>(*line));
        line_colors.push_back(std::get<3>(*line));
      }

      auto viewer = guik::viewer();
      viewer->update_drawable("odometry_factors", std::make_shared<glk::ThinLines>(line_vertices, line_colors), guik::VertexColor().set_alpha(factors_alpha));
    });
  });

  // Marginalized frames callback
  OdometryEstimationCallbacks::on_marginalized_frames.add([this](const std::vector<EstimationFrame::ConstPtr>& frames) {
    std::vector<int> marginalized_ids(frames.size());
    std::transform(frames.begin(), frames.end(), marginalized_ids.begin(), [](const EstimationFrame::ConstPtr& frame) { return frame->id; });

    const EstimationFrame::ConstPtr last_frame = frames.empty() ? nullptr : frames.back();

    invoke([this, marginalized_ids, last_frame] {
      if (last_frame) {
        trajectory->add_odom(last_frame->stamp, last_frame->T_world_sensor(), 2);
      }

      auto viewer = guik::LightViewer::instance();
      for (const int id : marginalized_ids) {
        viewer->remove_drawable("frame_" + std::to_string(id));
        viewer->remove_drawable("frame_coord_" + std::to_string(id));
        odometry_poses.erase(id);
      }
    });
  });

  // Marginalized keyframes callback
  OdometryEstimationCallbacks::on_marginalized_keyframes.add([this](const std::vector<EstimationFrame::ConstPtr>& keyframes) {
    invoke([this, keyframes] {
      auto viewer = guik::LightViewer::instance();
      for (const auto& keyframe : keyframes) {
        viewer->remove_drawable("odometry_keyframe_" + std::to_string(keyframe->id));
        viewer->remove_drawable("odometry_keyframe_coord_" + std::to_string(keyframe->id));
        odometry_poses.erase(keyframe->id);
      }
    });
  });

  /*** Submapping callbacks ***/
  // New keyframe callback
  SubMappingCallbacks::on_new_keyframe.add([this](int id, const EstimationFrame::ConstPtr& keyframe) {
    gtsam_points::PointCloud::ConstPtr frame = keyframe->frame;

    invoke([this, id, keyframe, frame] {
      auto viewer = guik::LightViewer::instance();
      auto sub_viewer = viewer->sub_viewer("submap");

      const Eigen::Vector4f color = glk::colormap_categoricalf(glk::COLORMAP::TURBO, id, 16);
      const auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(frame->points, frame->size());

      guik::FlatColor shader_setting(color);
      if (id == 0) {
        sub_viewer->clear_text();
        sub_viewer->clear_drawables();
        submap_keyframes.clear();

        const Eigen::Isometry3f T_world_key0 = keyframe->T_world_imu.cast<float>();
        Eigen::Isometry3f T_center_key0 = T_world_key0;
        T_center_key0.translation().setZero();

        shader_setting.add("T_world_key0", T_world_key0.matrix());
        shader_setting.add("T_center_key0", T_center_key0.matrix());
        shader_setting.add("model_matrix", T_center_key0.matrix());
      } else {
        auto drawable = sub_viewer->find_drawable("frame_0");
        if (drawable.first) {
          const Eigen::Isometry3f T_world_key0(drawable.first->cast<Eigen::Matrix4f>("T_world_key0"));
          const Eigen::Isometry3f T_center_key0(drawable.first->cast<Eigen::Matrix4f>("T_center_key0"));
          const Eigen::Isometry3f T_world_key1 = keyframe->T_world_imu.cast<float>();

          const Eigen::Isometry3f T_center_key1 = T_center_key0 * T_world_key0.inverse() * T_world_key1;
          shader_setting.add("model_matrix", T_center_key1.matrix());
        }
      }

      if (submap_keyframes.size() <= id) {
        submap_keyframes.resize(id + 1, Eigen::Isometry3f::Identity());
        submap_keyframes[id] = Eigen::Isometry3f(shader_setting.cast<Eigen::Matrix4f>("model_matrix"));
      }

      sub_viewer->update_drawable("frame_" + std::to_string(id), cloud_buffer, shader_setting);
      sub_viewer->update_drawable(
        "coord_" + std::to_string(id),
        glk::Primitives::coordinate_system(),
        guik::VertexColor(shader_setting.cast<Eigen::Matrix4f>("model_matrix") * Eigen::UniformScaling<float>(3.0f)));
    });
  });

  // Submap optimization callback
  SubMappingCallbacks::on_optimize_submap.add([this](gtsam::NonlinearFactorGraph& graph, gtsam::Values& values) {
    std::vector<std::pair<int, int>> factors;
    for (const auto& factor : graph) {
      if (factor->keys().size() != 2) {
        continue;
      }

      gtsam::Symbol symbol0(factor->keys()[0]);
      gtsam::Symbol symbol1(factor->keys()[1]);

      if (symbol0.chr() != 'x' || symbol1.chr() != 'x') {
        continue;
      }

      factors.push_back(std::make_pair(symbol0.index(), symbol1.index()));
    }

    invoke([this, factors] {
      auto viewer = guik::LightViewer::instance();
      auto sub_viewer = viewer->sub_viewer("submap");

      std::vector<Eigen::Vector3f> lines;
      for (const auto& factor : factors) {
        if (submap_keyframes.size() <= factor.first || submap_keyframes.size() <= factor.second) {
          continue;
        }

        lines.push_back(submap_keyframes[factor.first].translation());
        lines.push_back(submap_keyframes[factor.second].translation());
      }

      sub_viewer->update_drawable("factors", std::make_shared<glk::ThinLines>(lines), guik::FlatGreen().set_alpha(factors_alpha));
    });
  });

  // Submap optimization status callback
  SubMappingCallbacks::on_optimization_status.add([this](const gtsam_points::LevenbergMarquardtOptimizationStatus& status, const gtsam::Values& values) {
    logger->debug("--- submap optimization ---");
    logger->debug(status.to_short_string());
  });

  /*** Global mapping callbacks ***/
  // Insert submap callback
  GlobalMappingCallbacks::on_insert_submap.add([this](const SubMap::ConstPtr& submap) {
    // submap->T_world_origin can be updated in the global mapping
    // To safely pass it to the UI thread (pass-by-value of Eigen classes can crash) wrap it in a shared_ptr
    const std::shared_ptr<Eigen::Isometry3d> T_world_origin(new Eigen::Isometry3d(submap->T_world_origin));

    invoke([this, submap, T_world_origin] {
      const double stamp_endpoint_R = submap->odom_frames.back()->stamp;
      const Eigen::Isometry3d T_world_endpoint_R = (*T_world_origin) * submap->T_origin_endpoint_R;
      trajectory->update_anchor(stamp_endpoint_R, T_world_endpoint_R);

      auto viewer = guik::LightViewer::instance();
      auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(submap->frame->points, submap->frame->size());
      auto shader_setting = guik::Rainbow(T_world_origin->matrix().cast<float>());
      shader_setting.set_alpha(points_alpha);

      if (enable_partial_rendering) {
        cloud_buffer->enable_partial_rendering(partial_rendering_budget);
        shader_setting.add("dynamic_object", 0).make_transparent();
      }

      viewer->update_drawable("submap_" + std::to_string(submap->id), cloud_buffer, shader_setting);
    });
  });

  // Update submaps callback
  GlobalMappingCallbacks::on_update_submaps.add([this](const std::vector<SubMap::Ptr>& submaps) {
    const SubMap::ConstPtr latest_submap = submaps.back();

    std::vector<int> submap_ids(submaps.size());
    std::vector<Eigen::Isometry3f> submap_poses(submaps.size());
    for (int i = 0; i < submaps.size(); i++) {
      submap_ids[i] = submaps[i]->id;
      submap_poses[i] = submaps[i]->T_world_origin.cast<float>();
    }

    invoke([this, latest_submap, submap_ids, submap_poses] {
      auto viewer = guik::LightViewer::instance();

      std::vector<Eigen::Vector3f> submap_positions(submap_ids.size());
      last_submap_z = submap_poses.back().translation().z();

      for (int i = 0; i < submap_ids.size(); i++) {
        submap_positions[i] = submap_poses[i].translation();

        auto_z_range[0] = std::min<float>(auto_z_range[0], submap_poses[i].translation().z());
        auto_z_range[1] = std::max<float>(auto_z_range[1], submap_poses[i].translation().z());

        auto drawable = viewer->find_drawable("submap_" + std::to_string(submap_ids[i]));
        if (drawable.first) {
          drawable.first->add("model_matrix", submap_poses[i].matrix());
        }
        viewer->update_drawable("submap_coord_" + std::to_string(submap_ids[i]), glk::Primitives::coordinate_system(), guik::VertexColor(submap_poses[i]));
      }

      std::vector<unsigned int> indices;
      indices.reserve(global_between_factors.size() * 2);
      for (const auto& factor : global_between_factors) {
        if (factor.first < submap_ids.size() && factor.second < submap_ids.size()) {
          indices.push_back(factor.first);
          indices.push_back(factor.second);
        }
      }

      viewer->update_drawable("factors", std::make_shared<glk::ThinLines>(submap_positions, indices), guik::FlatGreen().set_alpha(factors_alpha));

      Eigen::Vector2f z = z_range;
      if (z_range_mode == 0) {
        z += auto_z_range;
      } else if (z_range_mode == 1) {
        z += Eigen::Vector2f::Constant(last_submap_z);
      }
      viewer->shader_setting().add<Eigen::Vector2f>("z_range", z);
    });
  });

  // Smoother update callback
  GlobalMappingCallbacks::on_smoother_update.add([this](gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) {
    std::vector<std::pair<int, int>> between_factors;
    for (const auto& factor : new_factors) {
      if (factor->keys().size() != 2) {
        continue;
      }

      const gtsam::Symbol symbol0(factor->keys()[0]);
      const gtsam::Symbol symbol1(factor->keys()[1]);
      if (symbol0.chr() != 'x' || symbol1.chr() != 'x') {
        continue;
      }

      between_factors.push_back(std::make_pair(symbol0.index(), symbol1.index()));
    }

    invoke([this, between_factors] { global_between_factors.insert(global_between_factors.end(), between_factors.begin(), between_factors.end()); });
  });

  // Smoother update result callback
  GlobalMappingCallbacks::on_smoother_update_result.add([this](gtsam_points::ISAM2Ext& isam2, const gtsam_points::ISAM2ResultExt& result) {
    logger->debug("--- iSAM2 update ({} values / {} factors) ---", result.num_values, result.num_factors);
    logger->debug(result.to_string());
  });
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
  viewer->register_ui_callback("main_menu_bar", [this] { main_menu(); }); // Register main_menu

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

  std::vector<const char*> current_color_modes = {"FLAT", "INTENSITY", "NORMAL"};
  ImGui::SetNextItemWidth(92);
  ImGui::Combo("color_mode", &current_color_mode, current_color_modes.data(), current_color_modes.size());

  ImGui::SameLine();
  if (ImGui::Button("Log")) {
    viewer->register_ui_callback("logging", guik::create_logger_ui(glim::get_ringbuffer_sink(), 0.5));
  }

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

  ImGui::Checkbox("submaps", &show_submaps);
  ImGui::SameLine();
  ImGui::Checkbox("factors", &show_factors);

  ImGui::Separator();

  std::vector<const char*> z_range_modes = {"AUTO", "LOCAL", "MANUAL"};
  bool update_z_range = false;

  ImGui::SetNextItemWidth(150);
  update_z_range |= ImGui::Combo("z_range_mode", &z_range_mode, z_range_modes.data(), z_range_modes.size());
  update_z_range |= ImGui::DragFloatRange2("z_min", &z_range[0], &z_range[1], 0.1f, -10000.0f, 10000.0f);
  if (update_z_range) {
    Eigen::Vector2f z = z_range;
    if (z_range_mode == 0) {
      z += auto_z_range;
    } else if (z_range_mode == 1) {
      z += Eigen::Vector2f::Constant(last_submap_z);
    }
    viewer->shader_setting().add<Eigen::Vector2f>("z_range", z);
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
}

void StandardViewer::main_menu() {
  bool load_map_triggered = false;
  bool close_map_triggered = false;

  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      if (ImGui::MenuItem("Load Map...")) {
        // Action is now handled after EndMainMenuBar to avoid issues with dialogs
        load_map_triggered = true;
      }
      if (ImGui::MenuItem("Close Map")) {
        // Placeholder for actual close map logic
        logger->info("Close Map selected (not implemented yet)");
        close_map_triggered = true;
      }
      ImGui::Separator();
      if (ImGui::MenuItem("Quit")) {
        // Using pfd for confirmation dialog, similar to OfflineViewer
        if (pfd::message("Confirm", "Quit?", pfd::choice::ok_cancel, pfd::icon::warning).result() == pfd::button::ok) {
          request_to_terminate = true;
        }
      }
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }

  // Handle actions outside ImGui rendering block
  if (load_map_triggered) {
    // Invoke the file dialog logic.
    // It's better to call pfd from the main thread via invoke if there are any potential threading issues with UI libraries.
    // However, pfd is often okay to call directly. For now, direct call for simplicity as per task.
    // If issues arise, this can be wrapped in invoke().
    auto selection = pfd::select_folder("Select GLIM dump directory", ".").result(); // Removed explicit pfd::opt::none
    if (!selection.empty()) {
      std::lock_guard<std::mutex> lock(selected_map_path_mutex);
      selected_map_path_for_gui = selection;
      logger->info("Selected map path: {}", selected_map_path_for_gui);
      // Further processing (like triggering map load) would happen here or be flagged
    } else {
      logger->info("Map selection cancelled.");
    }
  }

  if (close_map_triggered) {
    // Actual map closing logic will be called here in a future step
  }
}

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::StandardViewer();
}