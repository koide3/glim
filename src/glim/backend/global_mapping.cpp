#include <glim/backend/global_mapping.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam_ext/factors/loose_prior_factor.hpp>
#include <gtsam_ext/factors/rotate_vector3_factor.hpp>
#include <gtsam_ext/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_ext/optimizers/isam2_ext.hpp>
#include <gtsam_ext/cuda/stream_temp_buffer_roundrobin.hpp>

#include <gtsam_ext/optimizers/isam2_ext.hpp>
#include <gtsam_ext/cuda/stream_temp_buffer_roundrobin.hpp>

#include <glim/util/config.hpp>
#include <glim/common/imu_integration.hpp>
#include <glim/backend/callbacks.hpp>

namespace glim {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::E;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

using Callbacks = GlobalMappingCallbacks;

GlobalMapping::GlobalMapping() {
  Config config(GlobalConfig::get_config_path("config_backend"));

  max_implicit_loop_distance = config.param<double>("global_mapping", "max_implicit_loop_distance", 100.0);
  min_implicit_loop_overlap = config.param<double>("global_mapping", "min_implicit_loop_overlap", 0.1);

  imu_integration.reset(new IMUIntegration);
  stream_buffer_roundrobin.reset(new gtsam_ext::StreamTempBufferRoundRobin(64));

  new_values.reset(new gtsam::Values);
  new_factors.reset(new gtsam::NonlinearFactorGraph);

  gtsam::ISAM2Params isam2_params;
  if (config.param<bool>("global_mapping", "use_isam2_dogleg", false)) {
    gtsam::ISAM2DoglegParams dogleg_params;
    isam2_params.setOptimizationParams(dogleg_params);
  }
  isam2_params.setRelinearizeSkip(config.param<int>("global_mapping", "isam2_relinearize_skip", 1));
  isam2_params.setRelinearizeThreshold(config.param<double>("global_mapping", "isam2_relinearize_thresh", 0.1));
  isam2.reset(new gtsam_ext::ISAM2Ext(isam2_params));
}

GlobalMapping::~GlobalMapping() {}

void GlobalMapping::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Callbacks::on_insert_imu(stamp, linear_acc, angular_vel);
  imu_integration->insert_imu(stamp, linear_acc, angular_vel);
}

void GlobalMapping::insert_submap(const SubMap::Ptr& submap) {
  for (auto& frame : submap->frames) {
    frame->purge_frames();
  }
  for (auto& frame : submap->odom_frames) {
    frame->purge_frames();
  }

  const int current = submaps.size();
  const int last = current - 1;
  submaps.push_back(submap);

  gtsam::Pose3 current_T_world_imu = gtsam::Pose3::identity();
  gtsam::Pose3 last_T_world_imu = gtsam::Pose3::identity();

  if (current != 0) {
    if (isam2->valueExists(X(last))) {
      last_T_world_imu = isam2->calculateEstimate<gtsam::Pose3>(X(last));
    } else {
      last_T_world_imu = new_values->at<gtsam::Pose3>(X(last));
    }

    const Eigen::Isometry3d T_origin0_endpointR0 = submaps[last]->T_origin_endpoint_R;
    const Eigen::Isometry3d T_origin1_endpointL1 = submaps[current]->T_origin_endpoint_L;
    const Eigen::Isometry3d T_endpointR0_endpointL1 = submaps[last]->odom_frames.back()->T_world_imu.inverse() * submaps[current]->odom_frames.front()->T_world_imu;
    const Eigen::Isometry3d T_origin0_origin1 = T_origin0_endpointR0 * T_endpointR0_endpointL1 * T_origin1_endpointL1.inverse();

    current_T_world_imu = last_T_world_imu * gtsam::Pose3(T_origin0_origin1.matrix());
  } else {
    current_T_world_imu = gtsam::Pose3(submap->T_world_origin.matrix());
  }

  new_values->insert(X(current), current_T_world_imu);
  submap->T_world_origin = Eigen::Isometry3d(current_T_world_imu.matrix());
  Callbacks::on_insert_submap(submap);

  // Local velocities
  const gtsam::imuBias::ConstantBias imu_biasL(submap->frames.front()->imu_bias);
  const gtsam::imuBias::ConstantBias imu_biasR(submap->frames.back()->imu_bias);

  const Eigen::Vector3d v_origin_imuL = submap->T_world_origin.linear().inverse() * submap->frames.front()->v_world_imu;
  const Eigen::Vector3d v_origin_imuR = submap->T_world_origin.linear().inverse() * submap->frames.back()->v_world_imu;

  const auto prior_noise3 = gtsam::noiseModel::Isotropic::Precision(3, 1e6);
  const auto prior_noise6 = gtsam::noiseModel::Isotropic::Precision(6, 1e6);

  if(current > 0) {
    new_values->insert(E(current * 2), gtsam::Pose3((submap->T_world_origin * submap->T_origin_endpoint_L).matrix()));
    new_values->insert(V(current * 2), (submap->T_world_origin.linear() * v_origin_imuL).eval());
    new_values->insert(B(current * 2), imu_biasL);

    new_factors->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(current), E(current * 2), gtsam::Pose3(submap->T_origin_endpoint_L.matrix()), prior_noise6);
    new_factors->emplace_shared<gtsam_ext::RotateVector3Factor>(X(current), V(current * 2), v_origin_imuL, prior_noise3);
    new_factors->emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(current * 2), imu_biasL, prior_noise6);
    new_factors->emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(B(current * 2), B(current * 2 + 1), gtsam::imuBias::ConstantBias(), prior_noise6);
  }

  new_values->insert(E(current * 2 + 1), gtsam::Pose3((submap->T_world_origin * submap->T_origin_endpoint_R).matrix()));
  new_values->insert(V(current * 2 + 1), (submap->T_world_origin.linear() * v_origin_imuR).eval());
  new_values->insert(B(current * 2 + 1), imu_biasR);

  new_factors->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(current), E(current * 2 + 1), gtsam::Pose3(submap->T_origin_endpoint_R.matrix()), prior_noise6);
  new_factors->emplace_shared<gtsam::RotateVector3Factor>(X(current), V(current * 2 + 1), v_origin_imuR, prior_noise3);
  new_factors->emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(current * 2 + 1), imu_biasR, prior_noise6);

  if (current == 0) {
    new_factors->emplace_shared<gtsam_ext::LoosePriorFactor<gtsam::Pose3>>(X(0), current_T_world_imu, gtsam::noiseModel::Isotropic::Precision(6, 1e6));
  } else {
    new_factors->add(*create_matching_cost_factors(current));

    const double stampL = submaps[last]->frames.back()->stamp;
    const double stampR = submaps[current]->frames.front()->stamp;

    int num_integrated;
    const int imu_read_cursor = imu_integration->integrate_imu(stampL, stampR, imu_biasL, &num_integrated);
    imu_integration->erase_imu_data(imu_read_cursor);

    if(num_integrated < 2) {
      std::cerr << "warning: insufficient IMU data between submaps (global_mapping)!!" << std::endl;
      new_factors->emplace_shared<gtsam::BetweenFactor<gtsam::Vector3>>(V(last * 2 + 1), V(current * 2), gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Precision(3, 1.0));
    } else {
      new_factors->emplace_shared<gtsam::ImuFactor>(E(last * 2 + 1), V(last * 2 + 1), E(current * 2), V(current * 2), B(last * 2 + 1), imu_integration->integrated_measurements());
    }
  }

  Callbacks::on_smoother_update(*isam2, *new_factors, *new_values);
  auto result = isam2->update(*new_factors, *new_values);
  isam2->update();
  new_values.reset(new gtsam::Values);
  new_factors.reset(new gtsam::NonlinearFactorGraph);

  update_submaps();
  Callbacks::on_update_submaps(submaps);

  std::cout << result.to_string() << std::endl;
}

boost::shared_ptr<gtsam::NonlinearFactorGraph> GlobalMapping::create_matching_cost_factors(int current) const {
  auto factors = gtsam::make_shared<gtsam::NonlinearFactorGraph>();
  if(current == 0) {
    return factors;
  }

  const auto& current_submap = submaps.back();

  for (int i = 0; i < current; i++) {
    const double dist = (submaps[i]->T_world_origin.translation() - current_submap->T_world_origin.translation()).norm();
    if(dist > max_implicit_loop_distance) {
      continue;
    }

    const Eigen::Isometry3d delta = submaps[i]->T_world_origin.inverse() * current_submap->T_world_origin;
    const double overlap = current_submap->frame->overlap_gpu(submaps[i]->frame, delta);

    if(overlap < min_implicit_loop_overlap) {
      continue;
    }

    const auto stream_buffer = stream_buffer_roundrobin->get_stream_buffer();
    const auto& stream = stream_buffer.first;
    const auto& buffer = stream_buffer.second;

    factors->emplace_shared<gtsam_ext::IntegratedVGICPFactorGPU>(X(i), X(current), submaps[i]->frame, current_submap->frame, stream, buffer);
  }

  return factors;
}

void GlobalMapping::update_submaps() {
  for (int i = 0; i < submaps.size(); i++) {
    submaps[i]->T_world_origin = Eigen::Isometry3d(isam2->calculateEstimate<gtsam::Pose3>(X(i)).matrix());
  }
}

}  // namespace glim