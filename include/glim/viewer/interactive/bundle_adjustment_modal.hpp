#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam_points/types/point_cloud.hpp>
#include <glk/drawable.hpp>

#include <glim/mapping/sub_map.hpp>

namespace guik {
class GLCanvas;
class ProgressModal;
class ProgressInterface;
}  // namespace guik

namespace glim {

/**
 * @brief ImGUI modal for creating bundle adjustment factors
 */
class BundleAdjustmentModal {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BundleAdjustmentModal();
  ~BundleAdjustmentModal();

  void set_frames(const std::vector<SubMap::ConstPtr>& submaps, const std::vector<Eigen::Isometry3d>& submap_poses, const Eigen::Vector3d& center);

  gtsam::NonlinearFactor::shared_ptr run();

private:
  void update_indicator();
  std::vector<std::pair<int, int>> extract_points(double radius);
  Eigen::Vector3d calc_eigenvalues(const std::vector<std::pair<int, int>>& point_indices);
  double auto_radius(guik::ProgressInterface& progress);

  gtsam::NonlinearFactor::shared_ptr create_factor();

  void draw_canvas();

private:
  bool request_to_open;
  std::unique_ptr<guik::GLCanvas> canvas;
  std::unique_ptr<guik::ProgressModal> progress_modal;

  float radius;
  float min_radius;
  float max_radius;
  int min_points;
  int max_points;
  float plane_eps;

  int num_points;
  Eigen::Vector3d eigenvalues;

  Eigen::Vector4d center;
  std::vector<glim::SubMap::ConstPtr> submaps;
  std::vector<std::shared_ptr<const glk::Drawable>> submap_drawables;
  std::vector<Eigen::Isometry3d> submap_poses;
};

}  // namespace glim