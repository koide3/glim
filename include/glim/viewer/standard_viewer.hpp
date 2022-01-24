#pragma once

#include <mutex>
#include <atomic>
#include <thread>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glim {

class TrajectoryManager;
struct EstimationFrame;

class StandardViewer {
public:
  StandardViewer();
  virtual ~StandardViewer();

  bool ok() const;
  void wait();
  void stop();

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

  bool track;
  bool show_current;
  bool show_frontend_scans;
  bool show_frontend_keyframes;
  bool show_submaps;
  bool show_factors;

  std::unique_ptr<TrajectoryManager> trajectory;
  std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> submap_keyframes;

  std::vector<std::pair<int, int>> global_between_factors;

  std::mutex invoke_queue_mutex;
  std::vector<std::function<void()>> invoke_queue;
};
}