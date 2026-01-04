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

#include <glim/viewer/standard_viewer_mem.hpp>

namespace glim {

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

      std::vector<float> intensities;
      if (new_frame->raw_frame && new_frame->raw_frame->points.size() == new_frame->frame->size()) {
        intensities.resize(new_frame->raw_frame->intensities.size());
        std::copy(new_frame->raw_frame->intensities.begin(), new_frame->raw_frame->intensities.end(), intensities.begin());
      } else if (new_frame->frame->has_intensities()) {
        intensities.resize(new_frame->frame->size());
        std::copy(new_frame->frame->intensities, new_frame->frame->intensities + new_frame->frame->size(), intensities.begin());
      }

      if (!intensities.empty()) {
        for (const auto intensity : intensities) {
          intensity_dist.add(intensity);
        }

        cloud_buffer->add_colormap(intensities);
      }

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

      switch (odom_color_mode) {
        case 0:  // FLAT
          break;
        case 1:  // INTENSITY
          shader_setting.set_color_mode(guik::ColorMode::VERTEX_COLORMAP);
          shader_setting_rainbow.set_color_mode(guik::ColorMode::VERTEX_COLORMAP);
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

          shader_setting.set_color_mode(guik::ColorMode::VERTEX_COLORMAP);
          shader_setting_rainbow.set_color_mode(guik::ColorMode::VERTEX_COLORMAP);
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
          if (keyframe->frame->has_intensities()) {
            std::vector<float> intensities(keyframe->frame->intensities, keyframe->frame->intensities + keyframe->frame->size());
            cloud_buffer->add_colormap(intensities);
          }

          guik::Rainbow shader_setting(pose);
          switch (odom_color_mode) {
            case 0:  // FLAT
              break;
            case 1:  // INTENSITY
              shader_setting.set_color_mode(guik::ColorMode::VERTEX_COLORMAP);
              break;
            case 2:  // NORMAL
              break;
          }

          viewer->update_drawable(name, cloud_buffer, shader_setting);
        } else {
          switch (odom_color_mode) {
            case 0:
              drawable.first->set_color_mode(guik::ColorMode::RAINBOW);
              break;
            case 1:
              drawable.first->set_color_mode(guik::ColorMode::VERTEX_COLORMAP);
              break;
            case 2:
              drawable.first->set_color_mode(guik::ColorMode::RAINBOW);
              break;
          }

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
    std::vector<std::pair<std::weak_ptr<gtsam::NonlinearFactor>, FactorLineGetter>> new_factor_lines;
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

        const auto matching_factor = dynamic_cast<gtsam_points::IntegratedMatchingCostFactor*>(factor.get());
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
        const auto gpu_factor = dynamic_cast<gtsam_points::IntegratedVGICPFactorGPU*>(factor.get());
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
      if (frame->has_intensities()) {
        std::vector<float> intensities(frame->intensities, frame->intensities + frame->size());
        cloud_buffer->add_colormap(intensities);
      }

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
      if (submap->frame->has_intensities()) {
        std::vector<float> intensities(submap->frame->intensities, submap->frame->intensities + submap->frame->size());
        cloud_buffer->add_colormap(intensities);
      }

      auto shader_setting = guik::Rainbow(T_world_origin->matrix().cast<float>());
      switch (submap_color_mode) {
        case 0:  // RAINBOW
          shader_setting.set_color_mode(guik::ColorMode::RAINBOW);
          break;
        case 1:  // INTENSITY
          shader_setting.set_color_mode(guik::ColorMode::VERTEX_COLORMAP);
          break;
        case 2:  // COLOR
          shader_setting.set_color_mode(guik::ColorMode::VERTEX_COLOR);
          break;
      }

      shader_setting.set_alpha(points_alpha);

      if (enable_partial_rendering) {
        cloud_buffer->enable_partial_rendering(partial_rendering_budget);
        shader_setting.add("dynamic_object", 0).make_transparent();
      }

      total_gl_bytes += cloud_buffer->memory_usage();
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

    std::vector<SubMapMemoryStats> mem_stats;
    if (show_memory_stats) {
      mem_stats.reserve(submaps.size() - submap_memstats_count);
      for (int i = submap_memstats_count; i < submaps.size(); i++) {
        mem_stats.emplace_back(*submaps[i]);
      }
      submap_memstats_count = submaps.size();
    }

    invoke([this, latest_submap, submap_ids, submap_poses, mem_stats] {
      auto viewer = guik::LightViewer::instance();

      submap_memstats.insert(submap_memstats.end(), mem_stats.begin(), mem_stats.end());

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

    if (show_memory_stats) {
      std::vector<FactorMemoryStats> mem_stats;

      for (int i = global_factor_stats_count; i < isam2.getFactorsUnsafe().size(); i++) {
        FactorMemoryStats stats(isam2.getFactorsUnsafe()[i]);
        if (stats.cpu_bytes || stats.gpu_bytes) {
          mem_stats.emplace_back(stats);
        }
      }
      global_factor_stats_count = isam2.getFactorsUnsafe().size();

      invoke([this, mem_stats] { global_factor_memstats.insert(global_factor_memstats.end(), mem_stats.begin(), mem_stats.end()); });
    }
  });
}

}  // namespace glim