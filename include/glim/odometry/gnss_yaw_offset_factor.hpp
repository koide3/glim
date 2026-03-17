#pragma once

#include <cmath>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace glim {

/**
 * @brief GNSS position factor that jointly optimizes vehicle pose and ENU-to-world yaw offset.
 *
 * Variables:
 *   X = T_world_base  (Pose3)
 *   W = enu_yaw_world_x_  (double scalar, radians)
 *
 * Observation model:
 *   predicted  = T_world_base * lever_arm
 *   measured   = Rz(-W) * pos_enu + t_world_enu
 *   error (3D) = predicted - measured
 *
 * where Rz(-W) = [[cos(W), sin(W), 0], [-sin(W), cos(W), 0], [0, 0, 1]]
 * transforms an ENU-frame antenna position into the GLIM world frame.
 *
 * Analytical Jacobians are provided for both X (6-DoF Pose3) and W (scalar).
 */
class GNSSYawOffsetFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, double> {
public:
  using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, double>;

  GNSSYawOffsetFactor() = default;

  /**
   * @param pose_key     Key for T_world_base (Pose3 variable X(i))
   * @param yaw_key      Key for enu_yaw_world_x_ (double scalar W(0))
   * @param pos_enu      GNSS antenna position in ENU frame (metres)
   * @param t_world_enu  Fixed ENU-origin-to-world-origin translation vector
   * @param lever_arm    Lever arm: antenna position in base_link frame
   * @param model        3-D position noise model (use Robust + Diagonal)
   */
  GNSSYawOffsetFactor(
    gtsam::Key pose_key,
    gtsam::Key yaw_key,
    const gtsam::Point3& pos_enu,
    const gtsam::Point3& t_world_enu,
    const gtsam::Point3& lever_arm,
    const gtsam::SharedNoiseModel& model)
  : Base(model, pose_key, yaw_key),
    pos_enu_(pos_enu),
    t_world_enu_(t_world_enu),
    lever_arm_(lever_arm) {}

  ~GNSSYawOffsetFactor() override = default;

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new GNSSYawOffsetFactor(*this));
  }

  gtsam::Vector evaluateError(
    const gtsam::Pose3& T_world_base,
    const double& W,
    gtsam::OptionalMatrixType H1,
    gtsam::OptionalMatrixType H2) const override {

    // Rz(-W) maps ENU → world:  [[cos W, sin W, 0], [-sin W, cos W, 0], [0, 0, 1]]
    const double cW = std::cos(W);
    const double sW = std::sin(W);
    const double px = pos_enu_.x(), py = pos_enu_.y(), pz = pos_enu_.z();

    const gtsam::Point3 measured_world(
      cW * px + sW * py + t_world_enu_.x(),
      -sW * px + cW * py + t_world_enu_.y(),
      pz + t_world_enu_.z());

    // H1: Jacobian of (T_world_base * lever_arm) w.r.t. T_world_base (3×6)
    gtsam::Matrix36 H_pose;
    const gtsam::Point3 predicted_world =
      T_world_base.transformFrom(lever_arm_, H1 ? &H_pose : nullptr);

    if (H1) {
      *H1 = H_pose;
    }

    if (H2) {
      // d(error)/dW = -d(measured_world)/dW
      // d(measured_world.x)/dW = -sin(W)*px + cos(W)*py
      // d(measured_world.y)/dW = -cos(W)*px - sin(W)*py
      // d(measured_world.z)/dW = 0
      *H2 = (gtsam::Matrix31() <<
        -(-sW * px + cW * py),  // = sW*px - cW*py
        -(-cW * px - sW * py),  // = cW*px + sW*py
        0.0).finished();
    }

    return predicted_world - measured_world;
  }

private:
  gtsam::Point3 pos_enu_;
  gtsam::Point3 t_world_enu_;
  gtsam::Point3 lever_arm_;
};

}  // namespace glim
