#pragma once

#include <mutex>
#include <atomic>
#include <thread>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/util/extension_module.hpp>

namespace spdlog {
class logger;
}

namespace glim {

class TrajectoryManager;
struct EstimationFrame;

class StandardViewer : public ExtensionModule {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StandardViewer();
  virtual ~StandardViewer();

  virtual bool ok() const override;

  void invoke(const std::function<void()>& task);

private:
  Eigen::Isometry3f resolve_pose(const std::shared_ptr<const EstimationFrame>& frame);

  void set_callbacks();
  void viewer_loop();

  bool drawable_filter(const std::string& name);
  void drawable_selection();

private:
  std::atomic_bool request_to_terminate;
  std::atomic_bool kill_switch;
  std::thread thread;

  bool enable_partial_rendering;
  int partial_rendering_budget;

  bool track;
  bool show_current_coord;
  bool show_current_points;
  int current_color_mode;

  bool show_odometry_scans;
  bool show_odometry_keyframes;
  bool show_submaps;
  bool show_factors;

  bool show_odometry_status;
  int last_id;
  int last_num_points;
  std::pair<double, double> last_point_stamps;
  Eigen::Vector3d last_imu_vel;
  Eigen::Matrix<double, 6, 1> last_imu_bias;

  Eigen::Vector2f z_range;
  Eigen::Vector2f auto_z_range;

  std::unique_ptr<TrajectoryManager> trajectory;
  std::vector<Eigen::Isometry3f> submap_keyframes;

  std::vector<std::pair<int, int>> global_between_factors;

  std::mutex invoke_queue_mutex;
  std::vector<std::function<void()>> invoke_queue;
  
  // Logging
  std::shared_ptr<spdlog::logger> logger;
};
}