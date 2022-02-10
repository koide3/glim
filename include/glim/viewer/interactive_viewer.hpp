#pragma once

#include <atomic>
#include <memory>
#include <thread>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/backend/sub_map.hpp>
#include <glim/util/concurrent_vector.hpp>

namespace gtsam {
class Values;
class NonlinearFactor;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace gtsam_ext {
class ISAM2Ext;
}  // namespace gtsam_ext

namespace glim {

class ManualLoopCloseModal;
class BundleAdjustmentModal;

class InteractiveViewer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum class PickType { POINTS = 1, FRAME = (1 << 1), FACTOR = (1 << 2) };
  enum class FactorType { MATCHING_COST, BETWEEN, IMU };

  InteractiveViewer();
  virtual ~InteractiveViewer();

  bool ok() const;
  void wait();
  void stop();
  void clear();

protected:
  void viewer_loop();

  virtual void setup_ui() {}

  void invoke(const std::function<void()>& task);
  void drawable_selection();
  void on_click();
  void context_menu();
  void run_modals();

  void update_viewer();

  void globalmap_on_insert_submap(const SubMap::ConstPtr& submap);
  void globalmap_on_update_submaps(const std::vector<SubMap::Ptr>& updated_submaps);
  void globalmap_on_smoother_update(gtsam_ext::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values);

protected:
  std::atomic_bool request_to_clear;
  std::atomic_bool request_to_terminate;
  std::atomic_bool kill_switch;
  std::thread thread;

  // Tasks to be executed in the GUI thread
  std::mutex invoke_queue_mutex;
  std::vector<std::function<void()>> invoke_queue;

  // Visualization params
  float coord_scale;
  float sphere_scale;

  bool draw_points;
  bool draw_factors;
  bool draw_spheres;

  bool enable_partial_rendering;
  int partial_rendering_budget;

  // Click information
  Eigen::Vector4i right_clicked_info;
  Eigen::Vector3f right_clicked_pos;

  // GUI widgets
  std::unique_ptr<ManualLoopCloseModal> manual_loop_close_modal;
  std::unique_ptr<BundleAdjustmentModal> bundle_adjustment_modal;

  // Submaps
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> submap_poses;
  std::vector<SubMap::ConstPtr> submaps;

  // Factors
  std::vector<std::tuple<FactorType, std::uint64_t, std::uint64_t>> global_factors;

  // Factors to be inserted into the global mapping graph
  ConcurrentVector<boost::shared_ptr<gtsam::NonlinearFactor>> new_factors;
};
}