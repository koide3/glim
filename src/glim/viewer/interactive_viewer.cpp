#include <glim/viewer/interactive_viewer.hpp>

#include <glim/frontend/callbacks.hpp>
#include <glim/frontend/estimation_frame.hpp>

#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace glim {

class InteractiveViewer::Impl {
public:
  Impl() {
    set_callbacks();
    thread = std::thread([this] { viewer_loop(); });
  }
  ~Impl() {
    if (thread.joinable()) {
      thread.join();
    }
  }

  void viewer_loop() {
    auto viewer = guik::LightViewer::instance();
    viewer->enable_vsync();

    while (viewer->spin_once()) {
      std::lock_guard<std::mutex> lock(invoke_queue_mutex);
      for (const auto& task : invoke_queue) {
        task();
      }
      invoke_queue.clear();
    }
  }

  void invoke(const std::function<void()>& task) {
    std::lock_guard<std::mutex> lock(invoke_queue_mutex);
    invoke_queue.push_back(task);
  }

  void set_callbacks() {
    OdometryEstimationCallbacks::on_new_frame.add(std::bind(&Impl::frontend_new_frame, this, std::placeholders::_1));
    OdometryEstimationCallbacks::on_update_frames.add(std::bind(&Impl::frontend_on_update_frames, this, std::placeholders::_1));
    OdometryEstimationCallbacks::on_marginalized_frames.add(std::bind(&Impl::frontend_on_marginalized_frames, this, std::placeholders::_1));
  }

  void frontend_new_frame(const EstimationFrame::ConstPtr& new_frame) {
    invoke([this, new_frame] {
      auto viewer = guik::LightViewer::instance();
      auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(new_frame->frame->points, new_frame->frame->size());

      Eigen::Isometry3f pose = new_frame->T_world_imu.cast<float>();

      viewer->update_drawable("current", cloud_buffer, guik::FlatColor(1.0, 0.5, 0.0, 1.0, pose).add("point_scale", 2.0f));
      viewer->update_drawable("current_coord", glk::Primitives::coordinate_system(), guik::VertexColor(pose * Eigen::UniformScaling<float>(1.5f)));
      viewer->update_drawable("frame_" + std::to_string(new_frame->id), cloud_buffer, guik::Rainbow(pose));
    });
  }

  void frontend_on_update_frames(const std::vector<EstimationFrame::ConstPtr>& frames) {
    std::vector<int> frame_ids(frames.size());
    std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> frame_poses(frames.size());
    for (int i = 0; i < frames.size(); i++) {
      frame_ids[i] = frames[i]->id;
      frame_poses[i] = frames[i]->T_world_imu.cast<float>();
    }

    invoke([this, frame_ids, frame_poses] {
      auto viewer = guik::LightViewer::instance();
      for (int i = 0; i < frame_poses.size(); i++) {
        viewer->update_drawable("frontend_frame_" + std::to_string(i), glk::Primitives::coordinate_system(), guik::VertexColor(frame_poses[i]));

        auto drawable = viewer->find_drawable("frame_" + std::to_string(frame_ids[i]));
        if (drawable.first) {
          drawable.first->add("model_matrix", frame_poses[i].matrix());
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

private:
  std::thread thread;

  std::mutex invoke_queue_mutex;
  std::vector<std::function<void()>> invoke_queue;
};

InteractiveViewer::InteractiveViewer() {
  impl.reset(new Impl);
}

InteractiveViewer::~InteractiveViewer() {}

}  // namespace glim