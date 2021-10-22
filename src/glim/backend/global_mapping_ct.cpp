#include <glim/backend/global_mapping_ct.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam_ext/types/frame_cpu.hpp>
#include <gtsam_ext/factors/loose_prior_factor.hpp>
#include <gtsam_ext/factors/integrated_gicp_factor.hpp>
#include <gtsam_ext/factors/integrated_vgicp_factor.hpp>
#include <gtsam_ext/optimizers/isam2_ext.hpp>
#include <gtsam_ext/optimizers/levenberg_marquardt_ext.hpp>

#include <glim/util/config.hpp>
#include <glim/backend/callbacks.hpp>

namespace glim {

using Callbacks = GlobalMappingCallbacks;
using gtsam::symbol_shorthand::X;

GlobalMappingCT::GlobalMappingCT() {
  Config config(GlobalConfig::get_config_path("config_backend_ct"));
  randomsampling_rate = config.param<double>("global_mapping_ct", "randomsampling_rate", 0.05);
  max_implicit_loop_distance = config.param<double>("global_mapping_ct", "max_implicit_loop_distance", 30.0);
  min_implicit_loop_overlap = config.param<double>("global_mapping_ct", "min_implicit_loop_overlap", 0.25);

  new_values.reset(new gtsam::Values);
  new_factors.reset(new gtsam::NonlinearFactorGraph);

  gtsam::ISAM2Params isam2_params;
  if (config.param<bool>("global_mapping_ct", "use_isam2_dogleg", false)) {
    gtsam::ISAM2DoglegParams dogleg_params;
    isam2_params.setOptimizationParams(dogleg_params);
  }
  isam2_params.setRelinearizeSkip(config.param<int>("global_mapping_ct", "isam2_relinearize_skip", 1));
  isam2_params.setRelinearizeThreshold(config.param<double>("global_mapping_ct", "isam2_relinearize_thresh", 0.1));
  isam2.reset(new gtsam_ext::ISAM2Ext(isam2_params));
}

GlobalMappingCT::~GlobalMappingCT() {}

void GlobalMappingCT::insert_submap(const SubMap::Ptr& submap) {
  submap->drop_odom_frames();

  const int current = submaps.size();
  const int last = current - 1;
  submaps.push_back(submap);
  subsampled_submaps.push_back(gtsam_ext::random_sampling(submap->frame, randomsampling_rate, mt));

  gtsam::Pose3 current_T_world_submap = gtsam::Pose3::identity();
  gtsam::Pose3 last_T_world_submap = gtsam::Pose3::identity();

  if (current != 0) {
    if (isam2->valueExists(X(last))) {
      last_T_world_submap = isam2->calculateEstimate<gtsam::Pose3>(X(last));
    } else {
      last_T_world_submap = new_values->at<gtsam::Pose3>(X(last));
    }

    const Eigen::Isometry3d T_origin0_endpointR0 = submaps[last]->T_origin_endpoint_R;
    const Eigen::Isometry3d T_origin1_endpointL1 = submaps[current]->T_origin_endpoint_L;
    const Eigen::Isometry3d T_endpointR0_endpointL1 = submaps[last]->odom_frames.back()->T_world_imu.inverse() * submaps[current]->odom_frames.front()->T_world_imu;
    const Eigen::Isometry3d T_origin0_origin1 = T_origin0_endpointR0 * T_endpointR0_endpointL1 * T_origin1_endpointL1.inverse();

    current_T_world_submap = last_T_world_submap * gtsam::Pose3(T_origin0_origin1.matrix());
  } else {
    current_T_world_submap = gtsam::Pose3(submap->T_world_origin.matrix());
  }

  new_values->insert(X(current), current_T_world_submap);
  submap->T_world_origin = Eigen::Isometry3d(current_T_world_submap.matrix());
  Callbacks::on_insert_submap(submap);

  if (current == 0) {
    new_factors->emplace_shared<gtsam_ext::LoosePriorFactor<gtsam::Pose3>>(X(0), current_T_world_submap, gtsam::noiseModel::Isotropic::Precision(6, 1e6));
  } else {
    new_factors->add(*create_consecutive_factors(current));
    new_factors->add(*create_matching_cost_factors(current));
  }

  Callbacks::on_smoother_update(*isam2, *new_factors, *new_values);
  auto result = isam2->update(*new_factors, *new_values);
  new_values.reset(new gtsam::Values);
  new_factors.reset(new gtsam::NonlinearFactorGraph);
  Callbacks::on_smoother_update_result(*isam2, result);

  update_submaps();
  Callbacks::on_update_submaps(submaps);
}

void GlobalMappingCT::optimize() {
  if (isam2->empty()) {
    return;
  }
  auto result = isam2->update();
  Callbacks::on_smoother_update_result(*isam2, result);
}

boost::shared_ptr<gtsam::NonlinearFactorGraph> GlobalMappingCT::create_consecutive_factors(int current) const {
  auto factors = gtsam::make_shared<gtsam::NonlinearFactorGraph>();
  if (current == 0) {
    return factors;
  }

  const int last = current - 1;

  const gtsam::Pose3 init_delta = gtsam::Pose3((submaps[last]->T_world_origin.inverse() * submaps[current]->T_world_origin).matrix());

  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3::identity());
  values.insert(X(1), init_delta);

  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), gtsam::Pose3::identity(), gtsam::noiseModel::Isotropic::Precision(6, 1e6));

  auto factor = gtsam::make_shared<gtsam_ext::IntegratedGICPFactor>(X(0), X(1), submaps[last]->frame, submaps[current]->frame);
  graph.add(factor);

  gtsam_ext::LevenbergMarquardtExtParams lm_params;
  lm_params.setlambdaInitial(1e-12);
  lm_params.setMaxIterations(10);
  // lm_params.callback = [](const auto& status, const auto& values) { std::cout << status.to_string() << std::endl; };

  gtsam_ext::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);
  values = optimizer.optimize();

  const gtsam::Pose3 estimated_delta = values.at<gtsam::Pose3>(X(1));

  const auto linearized = factor->linearize(values);
  const auto H = linearized->hessianBlockDiagonal()[X(1)];

  factors->add(gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), estimated_delta, gtsam::noiseModel::Gaussian::Information(H)));
  return factors;
}

boost::shared_ptr<gtsam::NonlinearFactorGraph> GlobalMappingCT::create_matching_cost_factors(int current) const {
  auto factors = gtsam::make_shared<gtsam::NonlinearFactorGraph>();
  if (current == 0) {
    return factors;
  }

  const auto& current_submap = submaps.back();
  for (int i = 0; i < current; i++) {
    const double dist = (submaps[i]->T_world_origin.translation() - current_submap->T_world_origin.translation()).norm();
    if (dist > max_implicit_loop_distance) {
      continue;
    }

    const Eigen::Isometry3d delta = submaps[i]->T_world_origin.inverse() * current_submap->T_world_origin;
    const double overlap = current_submap->frame->overlap(submaps[i]->frame, delta);

    if (overlap < min_implicit_loop_overlap) {
      continue;
    }

    factors->emplace_shared<gtsam_ext::IntegratedVGICPFactor>(X(i), X(current), submaps[i]->frame, subsampled_submaps[current]);
  }

  return factors;
}

void GlobalMappingCT::update_submaps() {
  for (int i = 0; i < submaps.size(); i++) {
    submaps[i]->T_world_origin = Eigen::Isometry3d(isam2->calculateEstimate<gtsam::Pose3>(X(i)).matrix());
  }
}

}  // namespace glim