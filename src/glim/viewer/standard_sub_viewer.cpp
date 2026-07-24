#include <glim/viewer/standard_sub_viewer.hpp>

#include <glim/util/logging.hpp>
#include <glim/util/trajectory_manager.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/mapping/callbacks.hpp>

#include <glk/primitives/primitives.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace glim {

StandardSubViewer::StandardSubViewer() : logger(create_module_logger("subviewer")) {
  context_id = glim::CallbackContext::current();
  sub_viewer_name = "sub" + std::to_string(context_id);
  trajectory.reset(new TrajectoryManager);

  set_callbacks();
}

StandardSubViewer::~StandardSubViewer() {}

bool StandardSubViewer::ok() const {
  return true;
}

void StandardSubViewer::invoke(const std::function<void()>& task) {
  guik::viewer()->invoke(task);
}

Eigen::Isometry3f StandardSubViewer::resolve_pose(const std::shared_ptr<const EstimationFrame>& frame) {
  switch (frame->frame_id) {
    case FrameID::WORLD:
      return Eigen::Isometry3f::Identity();

    default:
      return trajectory->odom2world(frame->T_world_sensor()).cast<float>();
  }
}

void StandardSubViewer::set_callbacks() {
  IMUStateInitializationCallbacks::on_updated.add([this](const PreprocessedFrame::ConstPtr& frame, const Eigen::Isometry3d& T_odom_lidar_) {
    std::shared_ptr<Eigen::Isometry3d> T_odom_lidar(new Eigen::Isometry3d(T_odom_lidar_));
    invoke([this, frame, T_odom_lidar] {
      auto viewer = guik::viewer()->sub_viewer(sub_viewer_name);
      auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(frame->points);
      viewer->update_drawable("initialization_frame_" + guik::anon(), cloud_buffer, guik::FlatBlue(*T_odom_lidar));
      viewer->update_drawable("initialization_frame_current", cloud_buffer, guik::FlatOrange(*T_odom_lidar).set_point_scale(2.0f));
    });
  });

  // IMU state initialization termination callback
  IMUStateInitializationCallbacks::on_finished.add([this](const EstimationFrame::ConstPtr& frame) {
    invoke([this] {
      auto viewer = guik::viewer()->sub_viewer(sub_viewer_name);
      viewer->remove_drawable(std::regex("initialization.*"));
    });
  });

  // New frame callback
  OdometryEstimationCallbacks::on_new_frame.add([this](const EstimationFrame::ConstPtr& new_frame) {
    invoke([this, new_frame] {
      auto viewer = guik::viewer()->sub_viewer(sub_viewer_name);
      auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(new_frame->frame->points, new_frame->frame->size());

      trajectory->add_odom(new_frame->stamp, new_frame->T_world_sensor(), 1);
      const Eigen::Isometry3f pose = resolve_pose(new_frame);

      viewer->lookat(pose);

      guik::ShaderSetting shader_setting = guik::FlatColor(1.0f, 0.5f, 0.0f, 1.0f, pose);
      guik::ShaderSetting shader_setting_rainbow = guik::Rainbow(pose);

      viewer->update_drawable("current_frame", cloud_buffer, shader_setting.add("point_scale", 2.0f));
      viewer->update_drawable("current_coord", glk::Primitives::coordinate_system(), guik::VertexColor(pose * Eigen::UniformScaling<float>(1.5f)));
      viewer->update_drawable("frame_" + std::to_string(new_frame->id), cloud_buffer, shader_setting_rainbow);
    });
  });
}

bool StandardSubViewer::drawable_filter(const std::string& name) {
  return true;
}

void StandardSubViewer::drawable_selection() {}

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::StandardSubViewer();
}
