#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam_ext/types/frame.hpp>
#include <glk/drawable.hpp>

namespace guik {
class GLCanvas;
class ModelControl;
class ProgressModal;
class ProgressInterface;
}  // namespace guik

namespace glim {

/**
 * @brief ImGUI modal for manually creating loop factors
 */
class ManualLoopCloseModal {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ManualLoopCloseModal();
  ~ManualLoopCloseModal();

  void set_target(const gtsam::Key target_key, const gtsam_ext::Frame::ConstPtr& target, const Eigen::Isometry3d& target_pose);
  void set_source(const gtsam::Key target_key, const gtsam_ext::Frame::ConstPtr& source, const Eigen::Isometry3d& source_pose);

  gtsam::NonlinearFactor::shared_ptr run();

private:
  std::shared_ptr<Eigen::Isometry3d> align(guik::ProgressInterface& progress);
  gtsam::NonlinearFactor::shared_ptr create_factor();
  void draw_canvas();

private:
  bool request_to_open;
  std::unique_ptr<guik::GLCanvas> canvas;
  std::unique_ptr<guik::ProgressModal> progress_modal;
  std::unique_ptr<guik::ModelControl> model_control;

  float information_scale;
  float max_correspondence_distance;

  gtsam::Key target_key;
  gtsam::Key source_key;

  gtsam_ext::Frame::ConstPtr target;
  gtsam_ext::Frame::ConstPtr source;

  Eigen::Isometry3d target_pose;
  Eigen::Isometry3d source_pose;

  glk::Drawable::ConstPtr target_drawable;
  glk::Drawable::ConstPtr source_drawable;
};

}  // namespace glim