#include <glim/viewer/standard_viewer.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_ext/optimizers/isam2_result_ext.hpp>
#include <gtsam_ext/optimizers/levenberg_marquardt_optimization_status.hpp>

#include <glim/frontend/callbacks.hpp>
#include <glim/frontend/estimation_frame.hpp>
#include <glim/backend/callbacks.hpp>
#include <glim/util/trajectory_manager.hpp>

#include <glk/colormap.hpp>
#include <glk/thin_lines.hpp>
#include <glk/normal_distributions.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace glim {

class StandardViewer::Impl {
public:
  Impl() {
    kill_switch = false;
    request_to_terminate = false;

    show_current = true;
    show_frontend_scans = true;
    show_frontend_keyframes = true;
    show_submaps = true;

    trajectory.reset(new TrajectoryManager);

    set_callbacks();
    thread = std::thread([this] { viewer_loop(); });
  }

  ~Impl() {
    kill_switch = true;
    if (thread.joinable()) {
      thread.join();
    }
  }

  void viewer_loop() {
    auto viewer = guik::LightViewer::instance(Eigen::Vector2i(2560, 1440));
    viewer->enable_vsync();
    auto submap_viewer = viewer->sub_viewer("submap");
    submap_viewer->set_pos(Eigen::Vector2i(100, 800));
    submap_viewer->set_draw_xy_grid(false);
    submap_viewer->use_topdown_camera_control(80.0);

    viewer->add_drawable_filter("selection", [this](const std::string& name) { return drawable_filter(name); });
    viewer->register_ui_callback("selection", [this] { drawing_selection(); });

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

  bool drawable_filter(const std::string& name) {
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

    return true;
  }

  void drawing_selection() {
    ImGui::Begin("selection", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::Checkbox("current", &show_current);
    ImGui::Checkbox("frontend scans", &show_frontend_scans);
    ImGui::Checkbox("frontend keyframes", &show_frontend_keyframes);
    ImGui::Checkbox("submaps", &show_submaps);
    ImGui::End();
  }

  void invoke(const std::function<void()>& task) {
    if (kill_switch) {
      return;
    }

    std::lock_guard<std::mutex> lock(invoke_queue_mutex);
    invoke_queue.push_back(task);
  }

  void set_callbacks() {
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;

    OdometryEstimationCallbacks::on_new_frame.add(std::bind(&Impl::frontend_new_frame, this, _1));
    OdometryEstimationCallbacks::on_update_keyframes.add(std::bind(&Impl::frontend_on_update_keyframes, this, _1));
    OdometryEstimationCallbacks::on_marginalized_frames.add(std::bind(&Impl::frontend_on_marginalized_frames, this, _1));
    OdometryEstimationCallbacks::on_marginalized_keyframes.add(std::bind(&Impl::frontend_on_marginalized_keyframes, this, _1));

    SubMappingCallbacks::on_optimize_submap.add(std::bind(&Impl::submap_on_optimize_submap, this, _1, _2));
    SubMappingCallbacks::on_optimization_status.add(std::bind(&Impl::submap_on_optimization_status, this, _1, _2));
    SubMappingCallbacks::on_new_keyframe.add(std::bind(&Impl::submap_on_new_keyframe, this, _1, _2));
    // SubMappingCallbacks::on_new_submap.add(std::bind(&Impl::submap_on_new_submap, this, _1));

    GlobalMappingCallbacks::on_insert_submap.add(std::bind(&Impl::globalmap_on_insert_submap, this, _1));
    GlobalMappingCallbacks::on_update_submaps.add(std::bind(&Impl::globalmap_on_update_submaps, this, _1));
    GlobalMappingCallbacks::on_smoother_update.add(std::bind(&Impl::globalmap_on_smoother_update, this, _1, _2, _3));
    GlobalMappingCallbacks::on_smoother_update_result.add(std::bind(&Impl::globalmap_on_smoother_update_result, this, _1, _2));
  }

  Eigen::Isometry3f resolve_pose(const EstimationFrame::ConstPtr& frame) {
    if (frame->frame_id.empty()) {
      return Eigen::Isometry3f::Identity();
    }

    switch (frame->frame_id[0]) {
      default:
      case 'w':
        return Eigen::Isometry3f::Identity();

      case 'l':
        return trajectory->odom2world(frame->T_world_lidar).cast<float>();

      case 'i':
        return trajectory->odom2world(frame->T_world_imu).cast<float>();
    }
  }

  void frontend_new_frame(const EstimationFrame::ConstPtr& new_frame) {
    invoke([this, new_frame] {
      auto viewer = guik::LightViewer::instance();
      auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(new_frame->frame->points, new_frame->frame->size());

      trajectory->add_odom(new_frame->stamp, new_frame->T_world_imu);
      Eigen::Isometry3f pose = resolve_pose(new_frame);

      viewer->lookat(pose.translation());
      viewer->update_drawable("current_frame", cloud_buffer, guik::FlatColor(1.0, 0.5, 0.0, 1.0, pose).add("point_scale", 2.0f));
      viewer->update_drawable("current_coord", glk::Primitives::coordinate_system(), guik::VertexColor(pose * Eigen::UniformScaling<float>(1.5f)));
      viewer->update_drawable("frame_" + std::to_string(new_frame->id), cloud_buffer, guik::Rainbow(pose));
    });
  }

  void frontend_on_marginalized_frames(const std::vector<EstimationFrame::ConstPtr>& frames) {
    std::vector<int> marginalized_ids(frames.size());
    std::transform(frames.begin(), frames.end(), marginalized_ids.begin(), [](const EstimationFrame::ConstPtr& frame) { return frame->id; });

    invoke([this, marginalized_ids] {
      auto viewer = guik::LightViewer::instance();
      for (const int id : marginalized_ids) {
        viewer->remove_drawable("frame_" + std::to_string(id));
      }
    });
  }

  void frontend_on_update_keyframes(const std::vector<EstimationFrame::ConstPtr>& keyframes) {
    invoke([this, keyframes] {
      auto viewer = guik::LightViewer::instance();

      for (const auto& keyframe : keyframes) {
        Eigen::Isometry3f pose = resolve_pose(keyframe);

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
  }

  void frontend_on_marginalized_keyframes(const std::vector<EstimationFrame::ConstPtr>& keyframes) {
    invoke([this, keyframes] {
      auto viewer = guik::LightViewer::instance();
      for (const auto& keyframe : keyframes) {
        viewer->remove_drawable("frontend_keyframe_" + std::to_string(keyframe->id));
        viewer->remove_drawable("frontend_keyframe_coord_" + std::to_string(keyframe->id));
      }
    });
  }

  void submap_on_new_keyframe(int id, const EstimationFrame::ConstPtr& keyframe) {
    gtsam_ext::Frame::ConstPtr frame = keyframe->frame;

    invoke([this, id, keyframe, frame] {
      auto viewer = guik::LightViewer::instance();
      auto sub_viewer = viewer->sub_viewer("submap");

      const Eigen::Vector4f color = glk::colormap_categoricalf(glk::COLORMAP::TURBO, id, 16);
      auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(frame->points, frame->size());

      guik::FlatColor shader_setting(color);

      if (id == 0) {
        sub_viewer->clear_text();
        sub_viewer->clear_drawables();
        sub_keyframe_poses.clear();

        Eigen::Isometry3f T_world_key0 = keyframe->T_world_imu.cast<float>();
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

      if (sub_keyframe_poses.size() <= id) {
        sub_keyframe_poses.resize(id + 1, Eigen::Isometry3f::Identity());
        sub_keyframe_poses[id] = Eigen::Isometry3f(shader_setting.cast<Eigen::Matrix4f>("model_matrix"));
      }

      sub_viewer->update_drawable("frame_" + std::to_string(id), cloud_buffer, shader_setting);
      sub_viewer->update_drawable(
        "coord_" + std::to_string(id),
        glk::Primitives::coordinate_system(),
        guik::VertexColor(shader_setting.cast<Eigen::Matrix4f>("model_matrix") * Eigen::UniformScaling<float>(3.0f)));
    });
  }

  void submap_on_optimize_submap(gtsam::NonlinearFactorGraph& graph, gtsam::Values& values) {
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
        if (sub_keyframe_poses.size() <= factor.first || sub_keyframe_poses.size() <= factor.second) {
          continue;
        }

        lines.push_back(sub_keyframe_poses[factor.first].translation());
        lines.push_back(sub_keyframe_poses[factor.second].translation());
      }

      sub_viewer->update_drawable("factors", std::make_shared<glk::ThinLines>(lines), guik::FlatGreen());
    });
  }

  void submap_on_optimization_status(const gtsam_ext::LevenbergMarquardtOptimizationStatus& status, const gtsam::Values& values) {
    const std::string text = status.to_short_string();
    invoke([this, text] { guik::LightViewer::instance()->sub_viewer("submap")->append_text(text); });
  }

  void submap_on_new_submap(const SubMap::ConstPtr& submap) {
    invoke([this, submap] {
      auto viewer = guik::LightViewer::instance();

      std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> means(submap->frame->size());
      std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> covs(submap->frame->size());

      for (int i = 0; i < submap->frame->size(); i++) {
        means[i] = submap->frame->points[i].head<3>();
        covs[i] = submap->frame->covs[i].block<3, 3>(0, 0);
      }

      viewer->update_drawable("submap", std::make_shared<glk::NormalDistributions>(means, covs), guik::Rainbow());
    });
  }

  void globalmap_on_insert_submap(const SubMap::ConstPtr& submap) {
    // submap->T_world_origin can be updated in the global mapping
    // To safely pass it to the UI thread (pass-by-value of Eigen classes can crash) wrap it in a shared_ptr
    std::shared_ptr<Eigen::Isometry3d> T_world_origin(new Eigen::Isometry3d);
    *T_world_origin = submap->T_world_origin;

    invoke([this, submap, T_world_origin] {
      const double stamp_endpoint_R = submap->odom_frames.back()->stamp;
      const Eigen::Isometry3d T_world_endpoint_R = (*T_world_origin) * submap->T_origin_endpoint_R;
      trajectory->update_anchor(stamp_endpoint_R, T_world_endpoint_R);

      auto viewer = guik::LightViewer::instance();
      auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(submap->frame->points, submap->frame->size());
      viewer->update_drawable("submap_" + std::to_string(submap->id), cloud_buffer, guik::Rainbow(T_world_origin->matrix().cast<float>()));
    });
  }

  void globalmap_on_update_submaps(const std::vector<SubMap::Ptr>& submaps) {
    const SubMap::ConstPtr latest_submap = submaps.back();

    std::vector<int> submap_ids(submaps.size());
    std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> submap_poses(submaps.size());

    for (int i = 0; i < submaps.size(); i++) {
      submap_ids[i] = submaps[i]->id;
      submap_poses[i] = submaps[i]->T_world_origin.cast<float>();
    }

    invoke([this, latest_submap, submap_ids, submap_poses] {
      auto viewer = guik::LightViewer::instance();
      for (int i = 0; i < submap_ids.size(); i++) {
        auto drawable = viewer->find_drawable("submap_" + std::to_string(submap_ids[i]));
        if (drawable.first) {
          drawable.first->add("model_matrix", submap_poses[i].matrix());
        }
        viewer->update_drawable("submap_coord_" + std::to_string(submap_ids[i]), glk::Primitives::coordinate_system(), guik::VertexColor(submap_poses[i]));
      }

      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> factor_lines;
      factor_lines.reserve(global_between_factors.size() * 2);

      for (const auto& factor : global_between_factors) {
        if (factor.first >= submap_poses.size() || factor.second >= submap_poses.size()) {
          continue;
        }

        factor_lines.push_back(submap_poses[factor.first].translation());
        factor_lines.push_back(submap_poses[factor.second].translation());
      }

      viewer->update_drawable("factors", std::make_shared<glk::ThinLines>(factor_lines), guik::FlatGreen());
    });
  }

  void globalmap_on_smoother_update(gtsam_ext::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) {
    for (const auto& factor : new_factors) {
      if (factor->keys().size() != 2) {
        continue;
      }

      const gtsam::Symbol symbol0(factor->keys()[0]);
      const gtsam::Symbol symbol1(factor->keys()[1]);

      if (symbol0.chr() != 'x' || symbol1.chr() != 'x') {
        continue;
      }

      global_between_factors.push_back(std::make_pair(symbol0.index(), symbol1.index()));
    }
  }

  void globalmap_on_smoother_update_result(gtsam_ext::ISAM2Ext& isam2, const gtsam_ext::ISAM2ResultExt& result) {
    std::string text = (boost::format("--- iSAM2 update (%d values / %d factors) ---\n") % result.num_values % result.num_factors).str();
    text += result.to_string();

    invoke([this, text] { guik::LightViewer::instance()->append_text(text); });
  }

public:
  std::atomic_bool request_to_terminate;
  std::atomic_bool kill_switch;
  std::thread thread;

  bool show_current;
  bool show_frontend_scans;
  bool show_frontend_keyframes;
  bool show_submaps;

  std::unique_ptr<TrajectoryManager> trajectory;

  std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> sub_keyframe_poses;

  std::vector<std::pair<int, int>> global_between_factors;

  std::mutex invoke_queue_mutex;
  std::vector<std::function<void()>> invoke_queue;
};

StandardViewer::StandardViewer() {
  impl.reset(new Impl);
}

StandardViewer::~StandardViewer() {}

bool StandardViewer::ok() const {
  return !impl->request_to_terminate;
}

void StandardViewer::wait() {
  while (!impl->request_to_terminate) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  stop();
}

void StandardViewer::stop() {
  std::this_thread::sleep_for(std::chrono::seconds(1));

  impl->kill_switch = true;
  if (impl->thread.joinable()) {
    impl->thread.join();
  }
}

}  // namespace glim