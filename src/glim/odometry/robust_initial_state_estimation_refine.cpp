#include <glim/odometry/robust_initial_state_estimation.hpp>

#include <sstream>
#include <spdlog/spdlog.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>

#include <glim/common/imu_integration.hpp>
#include <glim/odometry/callbacks.hpp>

namespace glim {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::G;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

/**
 * @brief Refine the initial estimation using nonlinear optimization
 * @param gravity_aligned_values  Initial estimation values aligned with gravity. Expected to contain follows:
 *                                X(i) : Pose of the IMU in the world frame. Z axis should be aligned with upward direction (0, 0, 1)
 *                                V(i) : Velocity of the IMU in the world frame.
 *                                B(0) : IMU bias
 * @return gtsam::Values          Refined values
 * @note   It is often sufficient to use only the linear system solution, and using the nonlinear refinement may degrade the result.
 */
gtsam::Values RobustInitialStateEstimation::refine_nonlinear(const gtsam::Values& values) {
  gtsam::NonlinearFactorGraph graph;

  // Fix the translation and the yaw
  const gtsam::Pose3 T_world_imu0 = values.at<gtsam::Pose3>(X(0));
  const Eigen::Vector3d world_z = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d world_z_imu0 = T_world_imu0.rotation().unrotate(world_z);

  Eigen::Matrix<double, 6, 6> prior_info = Eigen::Matrix<double, 6, 6>::Identity() * 1e-3;
  prior_info.block<3, 3>(0, 0) += 1e3 * world_z_imu0 * world_z_imu0.transpose();
  prior_info.block<3, 3>(3, 3) += 1e3 * Eigen::Matrix3d::Identity();

  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), T_world_imu0, gtsam::noiseModel::Gaussian::Information(prior_info));

  // Fix the IMU bias (acc is not observable from the odometry)
  const gtsam::imuBias::ConstantBias imu_bias = values.at<gtsam::imuBias::ConstantBias>(B(0));
  graph.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(0), imu_bias, gtsam::noiseModel::Isotropic::Sigma(6, 1e-6));

  // Add odometry and IMU factors
  for (int i = 1; i < Ts_odom_lidar.size(); i++) {
    // Odometry factor
    const gtsam::Pose3 last_pose = values.at<gtsam::Pose3>(X(i - 1));
    const gtsam::Pose3 curr_pose = values.at<gtsam::Pose3>(X(i));
    const gtsam::Pose3 delta = last_pose.between(curr_pose);
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(i - 1), X(i), delta, gtsam::noiseModel::Isotropic::Sigma(6, 1e-3));

    // IMU factor
    const double prev_time = Ts_odom_lidar[i - 1].first;
    const double curr_time = Ts_odom_lidar[i].first;
    int num_integrated = 0;
    imu_integration->integrate_imu(prev_time, curr_time, imu_bias, &num_integrated);
    graph.emplace_shared<gtsam::ImuFactor>(X(i - 1), V(i - 1), X(i), V(i), B(0), imu_integration->integrated_measurements());
  }

  // Optimize!!
  gtsam::LevenbergMarquardtParams lm_params;
  auto optimized = gtsam::LevenbergMarquardtOptimizer(graph, values, lm_params).optimize();

  return optimized;
}

}  // namespace glim
