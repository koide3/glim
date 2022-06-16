#include <glim/frontend/odometry_estimation_cpu.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <gtsam_ext/ann/ivox.hpp>
#include <gtsam_ext/types/frame_cpu.hpp>
#include <gtsam_ext/factors/linear_damping_factor.hpp>
#include <gtsam_ext/factors/integrated_gicp_factor.hpp>
#include <gtsam_ext/factors/integrated_vgicp_factor.hpp>
#include <gtsam_ext/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_ext/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>

#include <glim/util/config.hpp>
#include <glim/util/console_colors.hpp>
#include <glim/common/imu_integration.hpp>
#include <glim/common/cloud_deskewing.hpp>
#include <glim/common/cloud_covariance_estimation.hpp>

#include <glim/frontend/callbacks.hpp>

namespace glim {

using Callbacks = OdometryEstimationCallbacks;

using gtsam::symbol_shorthand::B;  // IMU bias
using gtsam::symbol_shorthand::V;  // IMU velocity   (v_world_imu)
using gtsam::symbol_shorthand::X;  // IMU pose       (T_world_imu)

OdometryEstimationCPUParams::OdometryEstimationCPUParams() : OdometryEstimationIMUParams() {
  // frontend config
  Config config(GlobalConfig::get_config_path("config_frontend"));

  registration_type = config.param<std::string>("odometry_estimation", "registration_type", "VGICP");
  max_iterations = config.param<int>("odometry_estimation", "max_iterations", 5);
  lru_thresh = config.param<int>("odometry_estimation", "lru_thresh", 100);
  target_downsampling_rate = config.param<double>("odometry_estimation", "target_downsampling_rate", 0.1);

  ivox_resolution = config.param<double>("odometry_estimation", "ivox_resolution", 0.5);
  ivox_min_dist = config.param<double>("odometry_estimation", "ivox_min_dist", 0.1);

  vgicp_resolution = config.param<double>("odometry_estimation", "vgicp_resolution", 0.2);
  vgicp_voxelmap_levels = config.param<int>("odometry_estimation", "vgicp_voxelmap_levels", 2);
  vgicp_voxelmap_scaling_factor = config.param<double>("odometry_estimation", "vgicp_voxelmap_scaling_factor", 2.0);
}

OdometryEstimationCPUParams::~OdometryEstimationCPUParams() {}

OdometryEstimationCPU::OdometryEstimationCPU(const OdometryEstimationCPUParams& params) : OdometryEstimationIMU(std::make_unique<OdometryEstimationCPUParams>(params)) {
  last_target_update_pose.setIdentity();
  if (params.registration_type == "GICP") {
    target_ivox.reset(new gtsam_ext::iVox(params.ivox_resolution, params.ivox_min_dist, params.lru_thresh));
    target_ivox->set_neighbor_voxel_mode(1);
  } else if (params.registration_type == "VGICP") {
    target_voxelmaps.resize(params.vgicp_voxelmap_levels);
    for (int i = 0; i < params.vgicp_voxelmap_levels; i++) {
      const double resolution = params.vgicp_resolution * std::pow(params.vgicp_voxelmap_scaling_factor, i);
      target_voxelmaps[i] = std::make_shared<gtsam_ext::GaussianVoxelMapCPU>(resolution);
      target_voxelmaps[i]->set_lru_thresh(params.lru_thresh);
    }
  } else {
    std::cerr << console::bold_red << "error: unknown registration type for odometry_estimation_cpu (" << params.registration_type << ")" << console::reset << std::endl;
    abort();
  }
}

OdometryEstimationCPU::~OdometryEstimationCPU() {}

gtsam::NonlinearFactorGraph OdometryEstimationCPU::create_factors(const int current, const boost::shared_ptr<gtsam::ImuFactor>& imu_factor, gtsam::Values& new_values) {
  const auto params = static_cast<const OdometryEstimationCPUParams*>(this->params.get());
  const int last = current - 1;

  gtsam::NonlinearFactorGraph matching_cost_factors;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;

  const gtsam::Pose3 last_T_world_imu(frames[last]->T_world_imu.matrix());
  const gtsam::Vector3 last_v_world_imu = frames[last]->v_world_imu;
  const gtsam::imuBias::ConstantBias last_imu_bias(frames[last]->imu_bias);

  values.insert(X(last), gtsam::Pose3(last_T_world_imu));
  values.insert(X(current), gtsam::Pose3(frames[current]->T_world_imu.matrix()));

  values.insert(V(last), last_v_world_imu);
  values.insert(V(current), frames[current]->v_world_imu);
  values.insert(B(last), last_imu_bias);
  values.insert(B(current), gtsam::imuBias::ConstantBias(frames[current]->imu_bias));

  graph.add(imu_factor);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(last), last_T_world_imu, gtsam::noiseModel::Isotropic::Precision(6, 1e6));
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(last), last_v_world_imu, gtsam::noiseModel::Isotropic::Precision(3, 1e6));
  graph.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(last), last_imu_bias, gtsam::noiseModel::Isotropic::Precision(6, 1e6));
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(B(last), B(current), gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Isotropic::Precision(6, 1e6));

  // Create frame-to-model matching factor
  if (params->registration_type == "GICP") {
    auto gicp_factor =
      gtsam::make_shared<gtsam_ext::IntegratedGICPFactor_<gtsam_ext::iVox, gtsam_ext::Frame>>(gtsam::Pose3(), X(current), target_ivox, frames[current]->frame, target_ivox);
    gicp_factor->set_max_corresponding_distance(params->ivox_resolution * 2.0);
    gicp_factor->set_num_threads(params->num_threads);
    matching_cost_factors.add(gicp_factor);
  } else if (params->registration_type == "VGICP") {
    for (const auto& voxelmap : target_voxelmaps) {
      auto vgicp_factor = gtsam::make_shared<gtsam_ext::IntegratedVGICPFactor>(gtsam::Pose3(), X(current), voxelmap, frames[current]->frame);
      vgicp_factor->set_num_threads(params->num_threads);
      matching_cost_factors.add(vgicp_factor);
    }
  }

  graph.add(matching_cost_factors);

  gtsam_ext::LevenbergMarquardtExtParams lm_params;
  lm_params.setMaxIterations(params->max_iterations);
  lm_params.setAbsoluteErrorTol(0.1);

  gtsam::Pose3 last_pose = values.at<gtsam::Pose3>(X(current));
  lm_params.termination_criteria = [&](const gtsam::Values& values) {
    const gtsam::Pose3 current_pose = values.at<gtsam::Pose3>(X(current));
    const gtsam::Pose3 delta = last_pose.inverse() * current_pose;

    const double delta_t = delta.translation().norm();
    const double delta_r = Eigen::AngleAxisd(delta.rotation().matrix()).angle();
    last_pose = current_pose;

    if (delta_t < 1e-10 && delta_r < 1e-10) {
      // Maybe failed to solve the linear system
      return false;
    }

    // Convergence check
    return delta_t < 1e-3 && delta_r < 1e-3 * M_PI / 180.0;
  };

  // Optimize
  // lm_params.setDiagonalDamping(true);
  gtsam_ext::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);
  values = optimizer.optimize();

  const Eigen::Isometry3d T_world_imu = Eigen::Isometry3d(values.at<gtsam::Pose3>(X(current)).matrix());
  frames[current]->T_world_imu = T_world_imu;
  frames[current]->v_world_imu = values.at<gtsam::Vector3>(V(current));
  frames[current]->imu_bias = values.at<gtsam::imuBias::ConstantBias>(B(current)).vector();

  gtsam::NonlinearFactorGraph factors;
  const auto linearized = optimizer.last_linearized();

  // Get linearized matching cost factors
  for (int i = linearized->size() - matching_cost_factors.size(); i < linearized->size(); i++) {
    // factors.emplace_shared<gtsam::LinearContainerFactor>(linearized->at(i), values);
  }

  // TODO: Extract a relative pose covariance from a frame-to-model matching result? How?
  Eigen::Isometry3d T_last_current = frames[last]->T_world_imu.inverse() * T_world_imu;
  T_last_current.linear() = Eigen::Quaterniond(T_last_current.linear()).normalized().toRotationMatrix();
  factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), gtsam::Pose3(T_last_current.matrix()), gtsam::noiseModel::Isotropic::Precision(6, 1e6));

  new_values.insert_or_assign(X(current), gtsam::Pose3(frames[current]->T_world_imu.matrix()));
  new_values.insert_or_assign(V(current), frames[current]->v_world_imu);
  new_values.insert_or_assign(B(current), gtsam::imuBias::ConstantBias(frames[current]->imu_bias));

  return factors;
}

void OdometryEstimationCPU::fallback_smoother() {}

void OdometryEstimationCPU::update_frames(const int current, const gtsam::NonlinearFactorGraph& new_factors) {
  OdometryEstimationIMU::update_frames(current, new_factors);

  update_target(current);
}

void OdometryEstimationCPU::update_target(const int current) {
  const auto params = static_cast<const OdometryEstimationCPUParams*>(this->params.get());

  const Eigen::Isometry3d T_world_frame = frames[current]->T_world_imu;
  const gtsam_ext::Frame::ConstPtr frame = gtsam_ext::random_sampling(frames[current]->frame, params->target_downsampling_rate, mt);

  if (frames.size() >= 2) {
    const Eigen::Isometry3d delta = T_world_frame.inverse() * last_target_update_pose;
    const double delta_t = delta.translation().norm();
    const double delta_r = Eigen::AngleAxisd(delta.linear()).angle();

    if (delta_t < 0.05 && delta_r < 0.5 * M_PI / 180.0) {
      return;
    }
  }
  last_target_update_pose = T_world_frame;

  auto transformed = std::make_shared<gtsam_ext::FrameCPU>();
  transformed->num_points = frame->size();
  transformed->points_storage.resize(frame->size());
  transformed->covs_storage.resize(frame->size());
  transformed->points = transformed->points_storage.data();
  transformed->covs = transformed->covs_storage.data();

  for (int i = 0; i < frame->size(); i++) {
    transformed->points[i] = T_world_frame * frame->points[i];
    transformed->covs[i] = T_world_frame.matrix() * frame->covs[i] * T_world_frame.matrix().transpose();
  }

  if (params->registration_type == "GICP") {
    target_ivox->insert(*transformed);
  } else if (params->registration_type == "VGICP") {
    for (auto& target_voxelmap : target_voxelmaps) {
      target_voxelmap->insert(*transformed);
    }
  }

  // Update target point cloud (just for visualization).
  // This is not necessary for mapping and can safely be ablated.
  if (frames.size() % 50 == 0) {
    EstimationFrame::Ptr frame(new EstimationFrame);
    frame->id = frames.size() - 1;
    frame->stamp = frames.back()->stamp;

    frame->T_lidar_imu = frames.back()->T_lidar_imu;
    frame->T_world_lidar = frame->T_lidar_imu.inverse();
    frame->T_world_imu.setIdentity();

    frame->v_world_imu.setZero();
    frame->imu_bias.setZero();

    frame->frame_id = FrameID::IMU;

    if (params->registration_type == "GICP") {
      frame->frame = std::make_shared<gtsam_ext::FrameCPU>(target_ivox->voxel_points());
    } else if (params->registration_type == "VGICP") {
      std::vector<Eigen::Vector4d> points;
      for (const auto& voxel : target_voxelmaps[0]->voxels) {
        points.push_back(voxel.second->mean);
      }
      frame->frame = std::make_shared<gtsam_ext::FrameCPU>(points);
    }

    std::vector<EstimationFrame::ConstPtr> keyframes = {frame};
    Callbacks::on_update_keyframes(keyframes);

    if (target_ivox_frame) {
      std::vector<EstimationFrame::ConstPtr> marginalized_keyframes = {target_ivox_frame};
      Callbacks::on_marginalized_keyframes(marginalized_keyframes);
    }

    target_ivox_frame = frame;
  }
}


}  // namespace glim
