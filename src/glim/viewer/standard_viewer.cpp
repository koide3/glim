#include <glim/viewer/standard_viewer.hpp>

#include <gtsam/inference/Symbol.h>

#include <glim/frontend/callbacks.hpp>
#include <glim/frontend/estimation_frame.hpp>
#include <glim/backend/callbacks.hpp>
#include <glim/util/trajectory_manager.hpp>

#include <glk/colormap.hpp>
#include <glk/thin_lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace glim {

class StandardViewer::Impl {
public:
  Impl() {
    show_current = true;
    show_frontend_scans = true;
    show_submaps = true;

    trajectory.reset(new TrajectoryManager);

    set_callbacks();
    thread = std::thread([this] { viewer_loop(); });
  }
  ~Impl() {
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
    submap_viewer->use_topdown_camera_control(30.0);

    viewer->add_drawable_filter("selection", [this](const std::string& name) { return drawable_filter(name); });
    viewer->register_ui_callback("selection", [this] { drawing_selection(); });

    while (viewer->spin_once()) {
      std::lock_guard<std::mutex> lock(invoke_queue_mutex);
      for (const auto& task : invoke_queue) {
        task();
      }
      invoke_queue.clear();
    }
  }

  bool drawable_filter(const std::string& name) {
    if(!show_current && name == "current"){
      return false;
    }

    if(!show_frontend_scans && name.find("frame_") != std::string::npos) {
      return false;
    }

    if(!show_submaps && name.find("submap_") != std::string::npos) {
      return false;
    }

    return true;
  }

  void drawing_selection() {
    ImGui::Begin("selection");
    ImGui::Checkbox("current", &show_current);
    ImGui::Checkbox("frontend scans", &show_frontend_scans);
    ImGui::Checkbox("submaps", &show_submaps);
    ImGui::End();
  }

  void invoke(const std::function<void()>& task) {
    std::lock_guard<std::mutex> lock(invoke_queue_mutex);
    invoke_queue.push_back(task);
  }

  void set_callbacks() {
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;

    OdometryEstimationCallbacks::on_new_frame.add(std::bind(&Impl::frontend_new_frame, this, _1));
    // OdometryEstimationCallbacks::on_update_frames.add(std::bind(&Impl::frontend_on_update_frames, this, _1));
    OdometryEstimationCallbacks::on_marginalized_frames.add(std::bind(&Impl::frontend_on_marginalized_frames, this, _1));

    SubMappingCallbacks::on_new_keyframe.add(std::bind(&Impl::submap_on_new_keyframe, this, _1, _2));

    GlobalMappingCallbacks::on_insert_submap.add(std::bind(&Impl::globalmap_on_insert_submap, this, _1));
    GlobalMappingCallbacks::on_update_submaps.add(std::bind(&Impl::globalmap_on_update_submaps, this, _1));
    GlobalMappingCallbacks::on_smoother_update.add(std::bind(&Impl::globalmap_on_smoother_update, this, _1, _2, _3));
    GlobalMappingCallbacks::on_smoother_update_result.add(std::bind(&Impl::globalmap_on_smoother_update_result, this, _1, _2));
  }

  void frontend_new_frame(const EstimationFrame::ConstPtr& new_frame) {
    invoke([this, new_frame] {
      auto viewer = guik::LightViewer::instance();
      auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(new_frame->frame->points, new_frame->frame->size());

      trajectory->add_odom(new_frame->stamp, new_frame->T_world_imu);
      Eigen::Isometry3f pose = trajectory->odom2world(new_frame->T_world_imu).cast<float>();

      viewer->update_drawable("current", cloud_buffer, guik::FlatColor(1.0, 0.5, 0.0, 1.0, pose).add("point_scale", 2.0f));
      viewer->update_drawable("current_coord", glk::Primitives::coordinate_system(), guik::VertexColor(pose * Eigen::UniformScaling<float>(1.5f)));
      viewer->update_drawable("frame_" + std::to_string(new_frame->id), cloud_buffer, guik::Rainbow(pose));
    });
  }

  void frontend_on_update_frames(const std::vector<EstimationFrame::ConstPtr>& frames) {
    std::vector<int> frame_ids(frames.size());
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> frame_poses(frames.size());
    for (int i = 0; i < frames.size(); i++) {
      frame_ids[i] = frames[i]->id;
      frame_poses[i] = frames[i]->T_world_imu;
    }

    invoke([this, frame_ids, frame_poses] {
      auto viewer = guik::LightViewer::instance();
      for (int i = 0; i < frame_poses.size(); i++) {
        const Eigen::Isometry3f frame_pose = trajectory->odom2world(frame_poses[i]).cast<float>();

        viewer->update_drawable("frontend_frame_" + std::to_string(i), glk::Primitives::coordinate_system(), guik::VertexColor(frame_pose));

        auto drawable = viewer->find_drawable("frame_" + std::to_string(frame_ids[i]));
        if (drawable.first) {
          drawable.first->add("model_matrix", frame_pose.matrix());
        }
      }
    });
  }

  void frontend_on_update_keyframes(const std::vector<EstimationFrame::ConstPtr>& keyframes) {
    std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> poses(keyframes.size());
    for (int i = 0; i < keyframes.size(); i++) {
      poses[i] = keyframes[i]->T_world_imu.cast<float>();
    }

    invoke([this, poses] {
      auto viewer = guik::LightViewer::instance();
      for (int i = 0; i < poses.size(); i++) {
        viewer->update_drawable("frontend_keyframe_" + std::to_string(i), glk::Primitives::coordinate_system(), guik::FlatColor(0.4f, 0.4, 0.4f, 1.0f, poses[i]));
      }
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

  void submap_on_new_keyframe(int id, const EstimationFrame::ConstPtr& keyframe) {
    gtsam_ext::Frame::ConstPtr frame = keyframe->frame;

    invoke([this, id, keyframe, frame] {
      auto viewer = guik::LightViewer::instance();
      auto sub_viewer = viewer->sub_viewer("submap");

      const Eigen::Vector4f color = glk::colormap_categoricalf(glk::COLORMAP::TURBO, id, 16);
      auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(frame->points, frame->size());

      guik::FlatColor shader_setting(color);

      if (id == 0) {
        sub_viewer->clear_drawables();

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

      sub_viewer->update_drawable("frame_" + std::to_string(id), cloud_buffer, shader_setting);
    });
  }

  void globalmap_on_insert_submap(const SubMap::ConstPtr& submap) {
    invoke([this, submap] {
      const double stamp_endpoint_R = submap->odom_frames.back()->stamp;
      const Eigen::Isometry3d T_world_endpoint_R = submap->T_world_origin * submap->T_origin_endpoint_R;
      trajectory->update_anchor(stamp_endpoint_R, T_world_endpoint_R);

      auto viewer = guik::LightViewer::instance();
      auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(submap->frame->points, submap->frame->size());
      viewer->update_drawable("submap_" + std::to_string(submap->id), cloud_buffer, guik::Rainbow(submap->T_world_origin.matrix().cast<float>()));
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
      for (int i = 0; i<submap_ids.size(); i++) {
        auto drawable = viewer->find_drawable("submap_" + std::to_string(submap_ids[i]));
        if(drawable.first) {
          drawable.first->add("model_matrix", submap_poses[i].matrix());
        }
        viewer->update_drawable("submap_coord_" + std::to_string(submap_ids[i]), glk::Primitives::coordinate_system(), guik::VertexColor(submap_poses[i]));
      }

      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> factor_lines;
      factor_lines.reserve(global_between_factors.size() * 2);

      for(const auto& factor: global_between_factors) {
        if(factor.first >= submap_poses.size() || factor.second >= submap_poses.size()) {
          continue;
        }

        factor_lines.push_back(submap_poses[factor.first].translation());
        factor_lines.push_back(submap_poses[factor.second].translation());
      }

      viewer->update_drawable("factors", std::make_shared<glk::ThinLines>(factor_lines), guik::FlatGreen());
    });
  }

  void globalmap_on_smoother_update(gtsam_ext::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) {
    for(const auto& factor: new_factors) {
      if(factor->keys().size() != 2) {
        continue;
      }

      const gtsam::Symbol symbol0(factor->keys()[0]);
      const gtsam::Symbol symbol1(factor->keys()[1]);

      if(symbol0.chr() != 'x' || symbol1.chr() != 'x') {
        continue;
      }

      global_between_factors.push_back(std::make_pair(symbol0.index(), symbol1.index()));
    }
  }

  void globalmap_on_smoother_update_result(gtsam_ext::ISAM2Ext& isam2, const gtsam_ext::ISAM2ResultExt& result) {
    auto viewer = guik::LightViewer::instance();
    viewer->append_text(result.to_string());
  }

private:
  std::thread thread;

  bool show_current;
  bool show_frontend_scans;
  bool show_submaps;

  std::unique_ptr<TrajectoryManager> trajectory;
  std::vector<std::pair<int, int>> global_between_factors;

  std::mutex invoke_queue_mutex;
  std::vector<std::function<void()>> invoke_queue;
};

StandardViewer::StandardViewer() {
  impl.reset(new Impl);
}

StandardViewer::~StandardViewer() {}

}  // namespace glim