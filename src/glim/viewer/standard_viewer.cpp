#include <glim/viewer/standard_viewer.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_ext/optimizers/isam2_result_ext.hpp>
#include <gtsam_ext/optimizers/levenberg_marquardt_optimization_status.hpp>

#include <glim/common/callbacks.hpp>
#include <glim/frontend/callbacks.hpp>
#include <glim/frontend/estimation_frame.hpp>
#include <glim/backend/callbacks.hpp>
#include <glim/util/trajectory_manager.hpp>

#include <glk/colormap.hpp>
#include <glk/texture.hpp>
#include <glk/texture_opencv.hpp>
#include <glk/thin_lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/indexed_pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace glim {

StandardViewer::StandardViewer() {
  kill_switch = false;
  request_to_terminate = false;

  track = true;
  show_current = true;
  show_intensity = false;
  show_intensity = false;
  show_frontend_scans = true;
  show_frontend_keyframes = true;
  show_submaps = true;
  show_factors = true;

  show_frontend_status = false;
  last_id = last_num_points = 0;
  last_imu_vel.setZero();
  last_imu_bias.setZero();

  trajectory.reset(new TrajectoryManager);

  set_callbacks();
  thread = std::thread([this] { viewer_loop(); });
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

void StandardViewer::wait() {
  while (!request_to_terminate) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  stop();
}

void StandardViewer::stop() {
  std::this_thread::sleep_for(std::chrono::seconds(1));

  kill_switch = true;
  if (thread.joinable()) {
    thread.join();
  }
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

  CommonCallbacks::on_notification.add([this](NotificationLevel level, const std::string& message) { guik::LightViewer::instance()->append_text(message); });

  /*** Frontend callbacks ***/
  // New image callback
  OdometryEstimationCallbacks::on_insert_image.add([this](const double stamp, const cv::Mat& image) {
    invoke([this, image] {
      auto viewer = guik::LightViewer::instance();
      const auto texture = glk::create_texture(image);
      viewer->update_image("image", texture);
    });
  });

  // New frame callback
  OdometryEstimationCallbacks::on_new_frame.add([this](const EstimationFrame::ConstPtr& new_frame) {
    invoke([this, new_frame] {
      auto viewer = guik::LightViewer::instance();
      auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(new_frame->frame->points, new_frame->frame->size());

      if (!new_frame->raw_frame->intensities.empty()) {
        const double max_intensity = *std::max_element(new_frame->raw_frame->intensities.begin(), new_frame->raw_frame->intensities.end());
        cloud_buffer->add_intensity(glk::COLORMAP::TURBO, new_frame->raw_frame->intensities, 1.0 / 1.0);
      }

      last_id = new_frame->id;
      last_num_points = new_frame->frame->size();
      last_imu_vel = new_frame->v_world_imu;
      last_imu_bias = new_frame->imu_bias;

      trajectory->add_odom(new_frame->stamp, new_frame->T_world_sensor());
      const Eigen::Isometry3f pose = resolve_pose(new_frame);

      if (track) {
        viewer->lookat(pose.translation());
      }

      guik::ShaderSetting shader_setting = guik::FlatColor(1.0f, 0.5f, 0.0f, 1.0f, pose);
      if (show_intensity) {
        shader_setting.add("color_mode", guik::ColorMode::VERTEX_COLOR);
      }
      viewer->update_drawable("current_frame", cloud_buffer, shader_setting.add("point_scale", 2.0f));
      viewer->update_drawable("current_coord", glk::Primitives::coordinate_system(), guik::VertexColor(pose * Eigen::UniformScaling<float>(1.5f)));
      viewer->update_drawable("frame_" + std::to_string(new_frame->id), cloud_buffer, guik::VertexColor(pose));
    });
  });

  // Update keyframes callback
  OdometryEstimationCallbacks::on_update_keyframes.add([this](const std::vector<EstimationFrame::ConstPtr>& keyframes) {
    invoke([this, keyframes] {
      auto viewer = guik::LightViewer::instance();

      for (const auto& keyframe : keyframes) {
        const Eigen::Isometry3f pose = resolve_pose(keyframe);

        const std::string name = "frontend_keyframe_" + std::to_string(keyframe->id);
        auto drawable = viewer->find_drawable(name);
        if (drawable.first == nullptr) {
          auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(keyframe->frame->points, keyframe->frame->size());
          viewer->update_drawable(name, cloud_buffer, guik::Rainbow(pose));
        } else {
          drawable.first->add("model_matrix", pose.matrix());
        }

        viewer->update_drawable("frontend_keyframe_coord_" + std::to_string(keyframe->id), glk::Primitives::coordinate_system(), guik::VertexColor(pose));
      }
    });
  });

  // Marginalized frames callback
  OdometryEstimationCallbacks::on_marginalized_frames.add([this](const std::vector<EstimationFrame::ConstPtr>& frames) {
    std::vector<int> marginalized_ids(frames.size());
    std::transform(frames.begin(), frames.end(), marginalized_ids.begin(), [](const EstimationFrame::ConstPtr& frame) { return frame->id; });

    invoke([this, marginalized_ids] {
      auto viewer = guik::LightViewer::instance();
      for (const int id : marginalized_ids) {
        viewer->remove_drawable("frame_" + std::to_string(id));
      }
    });
  });

  // Marginalized keyframes callback
  OdometryEstimationCallbacks::on_marginalized_keyframes.add([this](const std::vector<EstimationFrame::ConstPtr>& keyframes) {
    invoke([this, keyframes] {
      auto viewer = guik::LightViewer::instance();
      for (const auto& keyframe : keyframes) {
        viewer->remove_drawable("frontend_keyframe_" + std::to_string(keyframe->id));
        viewer->remove_drawable("frontend_keyframe_coord_" + std::to_string(keyframe->id));
      }
    });
  });

  /*** Submapping callbacks ***/
  // New keyframe callback
  SubMappingCallbacks::on_new_keyframe.add([this](int id, const EstimationFrame::ConstPtr& keyframe) {
    gtsam_ext::Frame::ConstPtr frame = keyframe->frame;

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

      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> lines;
      for (const auto& factor : factors) {
        if (submap_keyframes.size() <= factor.first || submap_keyframes.size() <= factor.second) {
          continue;
        }

        lines.push_back(submap_keyframes[factor.first].translation());
        lines.push_back(submap_keyframes[factor.second].translation());
      }

      sub_viewer->update_drawable("factors", std::make_shared<glk::ThinLines>(lines), guik::FlatGreen());
    });
  });

  // Submap optimization status callback
  SubMappingCallbacks::on_optimization_status.add([this](const gtsam_ext::LevenbergMarquardtOptimizationStatus& status, const gtsam::Values& values) {
    const std::string text = status.to_short_string();
    invoke([this, text] { guik::LightViewer::instance()->sub_viewer("submap")->append_text(text); });
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
      viewer->update_drawable("submap_" + std::to_string(submap->id), cloud_buffer, guik::Rainbow(T_world_origin->matrix().cast<float>()));
    });
  });

  // Update submaps callback
  GlobalMappingCallbacks::on_update_submaps.add([this](const std::vector<SubMap::Ptr>& submaps) {
    const SubMap::ConstPtr latest_submap = submaps.back();

    std::vector<int> submap_ids(submaps.size());
    std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> submap_poses(submaps.size());
    for (int i = 0; i < submaps.size(); i++) {
      submap_ids[i] = submaps[i]->id;
      submap_poses[i] = submaps[i]->T_world_origin.cast<float>();
    }

    invoke([this, latest_submap, submap_ids, submap_poses] {
      auto viewer = guik::LightViewer::instance();

      Eigen::Vector2f z_range = Eigen::Vector2f(0.0f, 0.0f);
      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> submap_positions(submap_ids.size());

      for (int i = 0; i < submap_ids.size(); i++) {
        submap_positions[i] = submap_poses[i].translation();

        z_range[0] = std::min<float>(z_range[0], submap_poses[i].translation().z());
        z_range[1] = std::max<float>(z_range[1], submap_poses[i].translation().z());

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

      viewer->update_drawable("factors", std::make_shared<glk::ThinLines>(submap_positions, indices), guik::FlatGreen());
      viewer->shader_setting().add<Eigen::Vector2f>("z_range", z_range + Eigen::Vector2f(-2.0f, 4.0f));
    });
  });

  // Smoother update callback
  GlobalMappingCallbacks::on_smoother_update.add([this](gtsam_ext::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) {
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
  GlobalMappingCallbacks::on_smoother_update_result.add([this](gtsam_ext::ISAM2Ext& isam2, const gtsam_ext::ISAM2ResultExt& result) {
    std::string text = (boost::format("--- iSAM2 update (%d values / %d factors) ---\n") % result.num_values % result.num_factors).str();
    text += result.to_string();

    invoke([this, text] { guik::LightViewer::instance()->append_text(text); });
  });
}

void StandardViewer::viewer_loop() {
  auto viewer = guik::LightViewer::instance(Eigen::Vector2i(2560, 1440));
  viewer->enable_vsync();

  auto submap_viewer = viewer->sub_viewer("submap");
  submap_viewer->set_pos(Eigen::Vector2i(100, 800));
  submap_viewer->set_draw_xy_grid(false);
  submap_viewer->use_topdown_camera_control(80.0);

  viewer->register_drawable_filter("selection", [this](const std::string& name) { return drawable_filter(name); });
  viewer->register_ui_callback("selection", [this] { drawable_selection(); });

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

  guik::LightViewer::destroy();
}

bool StandardViewer::drawable_filter(const std::string& name) {
  const auto starts_with = [](const std::string& name, const std::string& pattern) {
    if (name.size() < pattern.size()) {
      return false;
    }

    return std::equal(pattern.begin(), pattern.end(), name.begin());
  };

  if (!show_current && starts_with(name, "current_")) {
    return false;
  }

  if (!show_frontend_scans && starts_with(name, "frame_")) {
    return false;
  }

  if (!show_frontend_keyframes && starts_with(name, "frontend_keyframe_")) {
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
  ImGui::Begin("selection", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

  if (ImGui::Checkbox("track", &track)) {
    if (track) {
      guik::LightViewer::instance()->reset_center();
    }
  }
  ImGui::SameLine();
  ImGui::Checkbox("current", &show_current);
  ImGui::SameLine();
  ImGui::Checkbox("intensity", &show_intensity);

  ImGui::Separator();
  bool show_frontend = show_frontend_scans || show_frontend_keyframes;
  if (ImGui::Checkbox("frontend", &show_frontend)) {
    show_frontend_scans = show_frontend_keyframes = show_frontend;
  }

  ImGui::SameLine();
  if (ImGui::Button("Status")) {
    show_frontend_status = true;
  }

  ImGui::Checkbox("scans", &show_frontend_scans);
  ImGui::SameLine();
  ImGui::Checkbox("keyframes", &show_frontend_keyframes);

  ImGui::Separator();
  bool show_backend = show_submaps || show_factors;
  if (ImGui::Checkbox("backend", &show_backend)) {
    show_submaps = show_factors = show_backend;
  }

  ImGui::Checkbox("submaps", &show_submaps);
  ImGui::SameLine();
  ImGui::Checkbox("factors", &show_factors);
  ImGui::End();

  if (show_frontend_status) {
    ImGui::Begin("frontend status", &show_frontend_status, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::Text("frame ID:%d", last_id);
    ImGui::Text("#points:%d", last_num_points);
    ImGui::Text("vel:%.3f %.3f %.3f", last_imu_vel[0], last_imu_vel[1], last_imu_vel[2]);
    ImGui::Text("bias:%.3f %.3f %.3f %.3f %.3f %.3f", last_imu_bias[0], last_imu_bias[1], last_imu_bias[2], last_imu_bias[3], last_imu_bias[4], last_imu_bias[5]);
    ImGui::End();
  }
}
}  // namespace glim