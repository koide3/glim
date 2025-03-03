#pragma once

#include <atomic>
#include <memory>
#include <thread>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/mapping/sub_map.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/concurrent_vector.hpp>

namespace spdlog {
class logger;
}

namespace gtsam {
class Values;
class NonlinearFactor;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace gtsam_points {
class ISAM2Ext;
class ISAM2ResultExt;
}  // namespace gtsam_points

namespace glim {

class TrajectoryManager;
class ManualLoopCloseModal;
class BundleAdjustmentModal;

class InteractiveViewer : public ExtensionModule {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum class PickType { POINTS = 1, FRAME = (1 << 1), FACTOR = (1 << 2) };
  enum class FactorType { MATCHING_COST, BETWEEN, IMU };

  InteractiveViewer();
  virtual ~InteractiveViewer();

  virtual bool ok() const override;
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

  void odometry_on_new_frame(const EstimationFrame::ConstPtr& new_frame);
  void globalmap_on_insert_submap(const SubMap::ConstPtr& submap);
  void globalmap_on_update_submaps(const std::vector<SubMap::Ptr>& updated_submaps);
  void globalmap_on_smoother_update(gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values);
  void globalmap_on_smoother_update_result(gtsam_points::ISAM2Ext& isam2, const gtsam_points::ISAM2ResultExt& result);

protected:
  std::atomic_bool request_to_clear;
  std::atomic_bool request_to_terminate;
  std::atomic_bool kill_switch;
  std::thread thread;

  // Process params
  int num_threads;

  // Tasks to be executed in the GUI thread
  std::mutex invoke_queue_mutex;
  std::vector<std::function<void()>> invoke_queue;

  // Visualization params
  int color_mode;

  float coord_scale;
  float sphere_scale;

  bool draw_current;
  bool draw_traj;
  bool draw_points;
  bool draw_factors;
  bool draw_spheres;

  float min_overlap;
  bool cont_optimize;

  bool enable_partial_rendering;
  int partial_rendering_budget;

  double point_size;
  bool point_size_metric;
  bool point_shape_circle;

  Eigen::Vector2f z_range;

  double points_alpha;
  double factors_alpha;

  std::atomic_bool needs_session_merge;

  // Click information
  Eigen::Vector4i right_clicked_info;
  Eigen::Vector3f right_clicked_pos;

  // GUI widgets
  std::unique_ptr<ManualLoopCloseModal> manual_loop_close_modal;
  std::unique_ptr<BundleAdjustmentModal> bundle_adjustment_modal;

  // Odometry
  std::unique_ptr<TrajectoryManager> trajectory;

  // Submaps
  std::vector<Eigen::Isometry3d> submap_poses;
  std::vector<SubMap::ConstPtr> submaps;

  // Factors
  std::vector<std::tuple<FactorType, std::uint64_t, std::uint64_t>> global_factors;

  // Factors to be inserted into the global mapping graph
  ConcurrentVector<boost::shared_ptr<gtsam::NonlinearFactor>> new_factors;

  // Logging
  std::shared_ptr<spdlog::logger> logger;
};
}  // namespace glim