#include <glim/viewer/interactive_viewer.hpp>

#include <atomic>
#include <thread>
#include <glim/common/callbacks.hpp>
#include <glim/backend/sub_map.hpp>
#include <glim/backend/callbacks.hpp>
#include <glim/util/concurrent_vector.hpp>

#include <glim/viewer/interactive/manual_loop_close_modal.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/progress_modal.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace glim {

using gtsam::symbol_shorthand::X;

class InteractiveViewer::Impl {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum class PickType { POINTS = 1, FRAME = (1 << 1), FACTOR = (1 << 2) };

  Impl() {
    kill_switch = false;
    request_to_terminate = false;

    coord_scale = 1.0f;
    sphere_scale = 1.0f;

    set_callbacks();
    thread = std::thread([this] { viewer_loop(); });
  }

  ~Impl() {
    kill_switch = true;
    if (thread.joinable()) {
      thread.join();
    }
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

    CommonCallbacks::on_notification.add(std::bind(&Impl::common_notification, this, _1, _2));
    GlobalMappingCallbacks::on_insert_submap.add(std::bind(&Impl::globalmap_on_insert_submap, this, _1));
    GlobalMappingCallbacks::on_update_submaps.add(std::bind(&Impl::globalmap_on_update_submaps, this, _1));
    GlobalMappingCallbacks::on_smoother_update.add(std::bind(&Impl::globalmap_on_smoother_update, this, _1, _2, _3));
  }

  void viewer_loop() {
    auto viewer = guik::LightViewer::instance(Eigen::Vector2i(2560, 1440));
    viewer->enable_info_buffer();
    viewer->enable_vsync();
    viewer->register_ui_callback("on_click", [this] { on_click(); });
    viewer->register_ui_callback("context_menu", [this] { context_menu(); });
    viewer->register_ui_callback("run_modals", [this] { run_modals(); });

    progress.reset(new guik::ProgressModal("progress"));
    manual_loop_close_modal.reset(new ManualLoopCloseModal);

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

    progress.reset();
    manual_loop_close_modal.reset();
    guik::LightViewer::destroy();
  }

  void on_click() {
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

  void context_menu() {
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

      ImGui::EndPopup();
    }
  }

  void run_modals() {
    std::vector<gtsam::NonlinearFactor::shared_ptr> factors;
    factors.push_back(manual_loop_close_modal->run());

    factors.erase(std::remove(factors.begin(), factors.end(), nullptr), factors.end());

    if (factors.size()) {
      new_factors.insert(factors);
      GlobalMappingCallbacks::request_to_optimize();
    }
  }

  void update_global_map() {
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
  }

  void common_notification(NotificationLevel level, const std::string& message) {
    std::cout << message << std::endl;
    // invoke([this, level, message] { guik::LightViewer::instance()->append_text(message); });
  }

  void globalmap_on_insert_submap(const SubMap::ConstPtr& submap) {
    std::shared_ptr<Eigen::Isometry3d> pose(new Eigen::Isometry3d(submap->T_world_origin));
    invoke([this, submap, pose] {
      submap_poses.push_back(*pose);
      submaps.push_back(submap);
      update_global_map();
    });
  }

  void globalmap_on_update_submaps(const std::vector<SubMap::Ptr>& updated_submaps) {
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses(updated_submaps.size());
    std::transform(updated_submaps.begin(), updated_submaps.end(), poses.begin(), [](const SubMap::ConstPtr& submap) { return submap->T_world_origin; });

    invoke([this, poses] {
      for (int i = 0; i < std::min(submaps.size(), poses.size()); i++) {
        submap_poses[i] = poses[i];
      }
      update_global_map();
    });
  }

  void globalmap_on_smoother_update(gtsam_ext::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) {
    auto factors = this->new_factors.get_all_and_clear();
    new_factors.add(factors);
  }

public:
  std::atomic_bool request_to_terminate;
  std::atomic_bool kill_switch;
  std::thread thread;

  float coord_scale;
  float sphere_scale;

  Eigen::Vector4i right_clicked_info;
  Eigen::Vector3f right_clicked_pos;

  std::unique_ptr<guik::ProgressModal> progress;

  std::unique_ptr<ManualLoopCloseModal> manual_loop_close_modal;

  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> submap_poses;
  std::vector<SubMap::ConstPtr> submaps;

  ConcurrentVector<gtsam::NonlinearFactor::shared_ptr> new_factors;

  std::mutex invoke_queue_mutex;
  std::vector<std::function<void()>> invoke_queue;
};

InteractiveViewer::InteractiveViewer() {
  impl.reset(new Impl);
}

InteractiveViewer::~InteractiveViewer() {}

bool InteractiveViewer::ok() const {
  return !impl->request_to_terminate;
}

void InteractiveViewer::wait() {
  while (!impl->request_to_terminate) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  stop();
}

void InteractiveViewer::stop() {
  std::this_thread::sleep_for(std::chrono::seconds(1));

  impl->kill_switch = true;
  if (impl->thread.joinable()) {
    impl->thread.join();
  }
}

}  // namespace glim
