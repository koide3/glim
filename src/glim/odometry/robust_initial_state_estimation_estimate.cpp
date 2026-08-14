#include <glim/odometry/robust_initial_state_estimation.hpp>

#include <sstream>
#include <spdlog/spdlog.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam_points/util/gtsam_migration.hpp>

#include <glim/util/config.hpp>
#include <glim/util/convert_to_string.hpp>
#include <glim/common/imu_integration.hpp>
#include <glim/odometry/callbacks.hpp>

namespace glim {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::G;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

// error = raw_dR.retract(H_rot_bw * bias) - measured_dR
class RotationBiasFactor : public gtsam::NoiseModelFactor1<gtsam::Vector3> {
public:
  RotationBiasFactor(gtsam::Key bias_key, const gtsam::Rot3& raw_dR, const gtsam::Rot3& measured_dR, const gtsam::Matrix33& H_rot_bw, const gtsam::SharedNoiseModel& model)
  : gtsam::NoiseModelFactor1<gtsam::Vector3>(model, bias_key),
    raw_dR(raw_dR),
    measured_dR(measured_dR),
    H_rot_bw(H_rot_bw) {}

  gtsam::Vector evaluateError(const gtsam::Vector3& bias, gtsam_points::OptionalMatrixType H = gtsam_points::NoneValue) const override {
    gtsam::Matrix33 H_corr_raw, H_err_corr;

    const gtsam::Rot3 corrected_dR = raw_dR.retract(H_rot_bw * bias, gtsam_points::NoneValue, H_corr_raw);
    const gtsam::Vector3 error = measured_dR.localCoordinates(corrected_dR, gtsam_points::NoneValue, H_err_corr);

    if (H) {
      *H = H_err_corr * H_corr_raw * H_rot_bw;
    }

    return error;
  }

private:
  gtsam::Rot3 raw_dR;
  gtsam::Rot3 measured_dR;
  gtsam::Matrix33 H_rot_bw;
};

Eigen::Vector3d RobustInitialStateEstimation::estimate_gyro_bias() {
  logger->debug("estimating gyro bias from {} odometry measurements", Ts_odom_lidar.size());

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  values.insert(0, gtsam::Vector3(0.0, 0.0, 0.0));

  // Estimate IMU angular velocity bias
  for (int i = 1; i < Ts_odom_lidar.size(); i++) {
    const double prev_time = Ts_odom_lidar[i - 1].first;
    const double curr_time = Ts_odom_lidar[i].first;
    const double dt = curr_time - prev_time;

    const Eigen::Isometry3d prev_T_odom_imu = Ts_odom_lidar[i - 1].second * T_lidar_imu;
    const Eigen::Isometry3d curr_T_odom_imu = Ts_odom_lidar[i].second * T_lidar_imu;

    const gtsam::Rot3 prev_R = gtsam::Rot3(prev_T_odom_imu.linear());
    const gtsam::Rot3 curr_R = gtsam::Rot3(curr_T_odom_imu.linear());
    const gtsam::Rot3 dR = prev_R.between(curr_R);

    int num_integrated = 0;
    imu_integration->integrate_imu(prev_time, curr_time, gtsam::imuBias::ConstantBias(), &num_integrated);
    logger->trace("integrated {} IMU measurements between {} and {}", num_integrated, prev_time, curr_time);

    const auto preint = imu_integration->integrated_measurements();
    const gtsam::Rot3 pred_dR = preint.deltaRij();
    const gtsam::Matrix33 H_rot_bw = preint.preintegrated_H_biasOmega().block<3, 3>(0, 0);
    gtsam::Matrix33 cov = preint.preintMeasCov().block<3, 3>(0, 0);

    if (num_integrated < 2) {
      logger->warn("not enough IMU measurements integrated between {} and {}. N={}", prev_time, curr_time, num_integrated);
      cov.setIdentity();
    }

    auto noise_model = gtsam::noiseModel::Gaussian::Covariance(cov);
    graph.emplace_shared<RotationBiasFactor>(0, pred_dR, dR, H_rot_bw, noise_model);
  }

  // Optimize!!
  gtsam::LevenbergMarquardtParams params;
  values = gtsam::LevenbergMarquardtOptimizer(graph, values, params).optimize();
  const gtsam::Vector3 gyro_bias = values.at<gtsam::Vector3>(0);

  logger->debug("estimated gyro bias: {}", convert_to_string(gyro_bias));

  return gyro_bias;
}

// error = pred_p - measured_p
// pred_p = prev_p + dt * prev_v + 0.5 * dt * dt * g + prev_R.rotate(delta_p + H_p_ba * acc_bias)
class LinearPositionFactor : public gtsam::NoiseModelFactor3<gtsam::Vector3, gtsam::Vector3, gtsam::Vector3> {
public:
  LinearPositionFactor(
    gtsam::Key gravity_key,
    gtsam::Key acc_bias_key,
    gtsam::Key prev_v_key,
    const gtsam::Vector3& measured_p,
    const gtsam::Vector3& prev_p,
    const gtsam::Rot3& prev_R,
    const gtsam::PreintegratedImuMeasurements& preint,
    double dt,
    const gtsam::SharedNoiseModel& noise_model)
  : gtsam::NoiseModelFactor3<gtsam::Vector3, gtsam::Vector3, gtsam::Vector3>(noise_model, {gravity_key, acc_bias_key, prev_v_key}),
    measured_p(measured_p),
    prev_p(prev_p),
    prev_R(prev_R),
    delta_p(preint.deltaPij()),
    H_p_ba(preint.preintegrated_H_biasAcc().block<3, 3>(3, 0)),
    dt(dt) {}

  gtsam::Vector evaluateError(
    const gtsam::Vector3& gravity,
    const gtsam::Vector3& acc_bias,
    const gtsam::Vector3& prev_v,
    gtsam_points::OptionalMatrixType H_gravity = gtsam_points::NoneValue,
    gtsam_points::OptionalMatrixType H_acc_bias = gtsam_points::NoneValue,
    gtsam_points::OptionalMatrixType H_prev_v = gtsam_points::NoneValue) const override {
    //
    const gtsam::Vector3 pred_p =  //
      prev_p +                     //
      dt * prev_v +                //
      0.5 * dt * dt * gravity +    //
      prev_R.rotate(delta_p + H_p_ba * acc_bias);

    if (H_gravity) {
      *H_gravity = 0.5 * dt * dt * gtsam::Matrix33::Identity();
    }
    if (H_acc_bias) {
      *H_acc_bias = prev_R.matrix() * H_p_ba;
    }
    if (H_prev_v) {
      *H_prev_v = dt * gtsam::Matrix33::Identity();
    }

    return pred_p - measured_p;
  }

public:
  const gtsam::Vector3 measured_p;
  const gtsam::Vector3 prev_p;
  const gtsam::Rot3 prev_R;

  const gtsam::Vector3 delta_p;
  const gtsam::Matrix33 H_p_ba;
  const double dt;
};

// error = pred_v - curr_v
// pred_v = prev_v + dt * g + prev_R.rotate(delta_v + H_v_ba * acc_bias)
class LinearVelocityFactor : public gtsam::NoiseModelFactor4<gtsam::Vector3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector3> {
public:
  LinearVelocityFactor(
    gtsam::Key gravity_key,
    gtsam::Key acc_bias_key,
    gtsam::Key prev_v_key,
    gtsam::Key curr_v_key,
    const gtsam::Rot3& prev_R,
    const gtsam::PreintegratedImuMeasurements& preint,
    double dt,
    gtsam::SharedNoiseModel noise_model)
  : gtsam::NoiseModelFactor4<gtsam::Vector3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector3>(noise_model, {gravity_key, acc_bias_key, prev_v_key, curr_v_key}),
    prev_R(prev_R),
    delta_v(preint.deltaVij()),
    H_v_ba(preint.preintegrated_H_biasAcc().block<3, 3>(6, 0)),
    dt(dt) {}

  gtsam::Vector evaluateError(
    const gtsam::Vector3& gravity,
    const gtsam::Vector3& acc_bias,
    const gtsam::Vector3& prev_v,
    const gtsam::Vector3& curr_v,
    gtsam_points::OptionalMatrixType H_gravity = gtsam_points::NoneValue,
    gtsam_points::OptionalMatrixType H_acc_bias = gtsam_points::NoneValue,
    gtsam_points::OptionalMatrixType H_prev_v = gtsam_points::NoneValue,
    gtsam_points::OptionalMatrixType H_curr_v = gtsam_points::NoneValue) const override {
    //
    const gtsam::Vector3 pred_v =  //
      prev_v +                     //
      dt * gravity +               //
      prev_R.rotate(delta_v + H_v_ba * acc_bias);

    if (H_gravity) {
      *H_gravity = dt * gtsam::Matrix33::Identity();
    }
    if (H_acc_bias) {
      *H_acc_bias = prev_R.matrix() * H_v_ba;
    }
    if (H_prev_v) {
      *H_prev_v = gtsam::Matrix33::Identity();
    }
    if (H_curr_v) {
      *H_curr_v = -gtsam::Matrix33::Identity();
    }

    return pred_v - curr_v;
  }

public:
  const gtsam::Rot3 prev_R;
  const gtsam::Vector3 delta_v;
  const gtsam::Matrix33 H_v_ba;
  const double dt;
};

// error = |g| - measured_norm
class VectorNormFactor : public gtsam::NoiseModelFactor1<gtsam::Vector3> {
public:
  VectorNormFactor(gtsam::Key key, double measured_norm, const gtsam::SharedNoiseModel& noise_model)
  : gtsam::NoiseModelFactor1<gtsam::Vector3>(noise_model, key),
    measured_norm(measured_norm) {}

  gtsam::Vector evaluateError(const gtsam::Vector3& vec, gtsam_points::OptionalMatrixType H = gtsam_points::NoneValue) const override {
    const double norm = vec.norm();
    if (H) {
      if (norm < 1e-6) {
        *H = gtsam::Matrix13::Zero();
      } else {
        *H = vec.transpose() / norm;
      }
    }
    return gtsam::Vector1(norm - measured_norm);
  }

public:
  const double measured_norm;
};

gtsam::Values RobustInitialStateEstimation::estimate_linear_system(const gtsam::Vector3& gyro_bias) {
  if (Ts_odom_lidar.size() < 6) {
    logger->warn("Not enough odometry poses for linear system estimation");
    return gtsam::Values();
  }
  logger->debug("estimating linear system from {} odometry measurements", Ts_odom_lidar.size());

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;

  values.insert(B(0), gtsam::Vector3(0.0, 0.0, 0.0));
  values.insert(G(0), gtsam::Vector3(0.0, 0.0, 0.0));
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(B(0), gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Sigma(3, 0.01));

  // Estimate IMU angular velocity bias
  for (int i = 0; i < Ts_odom_lidar.size(); i++) {
    values.insert(V(i), gtsam::Vector3(0.0, 0.0, 0.0));
    if (i == 0) {
      continue;
    }

    const Eigen::Isometry3d prev_T_odom_imu = Ts_odom_lidar[i - 1].second * T_lidar_imu;
    const Eigen::Isometry3d curr_T_odom_imu = Ts_odom_lidar[i].second * T_lidar_imu;

    const gtsam::Rot3 prev_R(prev_T_odom_imu.linear());
    const gtsam::Vector3 prev_p = prev_T_odom_imu.translation();

    const gtsam::Rot3 curr_R(curr_T_odom_imu.linear());
    const gtsam::Vector3 curr_p = curr_T_odom_imu.translation();

    const double prev_time = Ts_odom_lidar[i - 1].first;
    const double curr_time = Ts_odom_lidar[i].first;
    const double dt = curr_time - prev_time;

    int num_integrated = 0;
    gtsam::imuBias::ConstantBias bias(gtsam::Vector3(0.0, 0.0, 0.0), gyro_bias);
    imu_integration->integrate_imu(prev_time, curr_time, bias, &num_integrated);
    logger->trace("integrated {} IMU measurements between {} and {}", num_integrated, prev_time, curr_time);

    const auto preint = imu_integration->integrated_measurements();
    graph.emplace_shared<LinearPositionFactor>(G(0), B(0), V(i - 1), curr_p, prev_p, prev_R, preint, dt, gtsam::noiseModel::Isotropic::Sigma(3, 1.0));
    graph.emplace_shared<LinearVelocityFactor>(G(0), B(0), V(i - 1), V(i), prev_R, preint, dt, gtsam::noiseModel::Isotropic::Sigma(3, 1e-3));
  }

  gtsam::LevenbergMarquardtParams params;
  values = gtsam::LevenbergMarquardtOptimizer(graph, values, params).optimize();

  if (std::abs(values.at<gtsam::Vector3>(G(0)).norm() - 9.81) > 1.0) {
    logger->warn("Estimated gravity norm is too far from 9.81 m/s^2: {}", values.at<gtsam::Vector3>(G(0)).norm());
  }

  // Add a prior factor to constrain the gravity norm to be 9.81 m/s^2
  graph.emplace_shared<VectorNormFactor>(G(0), 9.81, gtsam::noiseModel::Isotropic::Sigma(1, 1e-3));
  values = gtsam::LevenbergMarquardtOptimizer(graph, values, params).optimize();

  const gtsam::Vector3 estimated_gravity = values.at<gtsam::Vector3>(G(0));
  const gtsam::Vector3 estimated_acc_bias = values.at<gtsam::Vector3>(B(0));
  logger->debug("estimated gravity: {} (norm={})", convert_to_string(estimated_gravity), estimated_gravity.norm());
  logger->debug("estimated accelerometer bias: {}", convert_to_string(estimated_acc_bias));

  return values;
}

//
}  // namespace glim
