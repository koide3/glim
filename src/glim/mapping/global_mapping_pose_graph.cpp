#include <glim/mapping/global_mapping_pose_graph.hpp>

#include <spdlog/spdlog.h>
#include <boost/filesystem.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam_points/ann/kdtree.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/factors/linear_damping_factor.hpp>
#include <gtsam_points/optimizers/isam2_ext.hpp>
#include <gtsam_points/optimizers/isam2_ext_dummy.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/util/parallelism.hpp>

#include <glim/util/config.hpp>
#include <glim/util/serialization.hpp>
#include <glim/mapping/callbacks.hpp>

#ifdef GTSAM_USE_TBB
#include <tbb/task_arena.h>
#endif

namespace glim {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::E;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

using Callbacks = GlobalMappingCallbacks;

GlobalMappingPoseGraphParams::GlobalMappingPoseGraphParams() {
  Config config(GlobalConfig::get_config_path("config_global_mapping"));

  enable_optimization = config.param<bool>("global_mapping", "enable_optimization", true);
  registration_type = config.param<std::string>("global_mapping", "registration_type", "GICP");

  min_travel_dist = config.param<double>("global_mapping", "min_travel_dist", 100.0);
  max_neighbor_dist = config.param<double>("global_mapping", "max_neighbor_dist", 10.0);
  min_inliear_fraction = config.param<double>("global_mapping", "min_inliear_fraction", 0.5);

  subsample_target = config.param<int>("global_mapping", "subsample_target", 10000);
  subsample_rate = config.param<double>("global_mapping", "subsample_rate", 0.1);
  gicp_max_correspondence_dist = config.param<double>("global_mapping", "gicp_max_correspondence_dist", 2.0);
  vgicp_voxel_resolution = config.param<double>("global_mapping", "vgicp_voxel_resolution", 2.0);

  odom_factor_stddev = config.param<double>("global_mapping", "odom_factor_stddev", 1e-3);
  loop_factor_stddev = config.param<double>("global_mapping", "loop_factor_stddev", 0.1);
  loop_factor_robust_width = config.param<double>("global_mapping", "loop_factor_robust_width", 1.0);

  loop_candidate_buffer_size = config.param<int>("global_mapping", "loop_candidate_buffer_size", 100);
  loop_candidate_eval_per_thread = config.param<int>("global_mapping", "loop_candidate_eval_per_thread", 2);

  use_isam2_dogleg = config.param<bool>("global_mapping", "use_isam2_dogleg", false);
  isam2_relinearize_skip = config.param<int>("global_mapping", "isam2_relinearize_skip", 1);
  isam2_relinearize_thresh = config.param<double>("global_mapping", "isam2_relinearize_thresh", 0.1);

  init_pose_damping_scale = config.param<double>("global_mapping", "init_pose_damping_scale", 1e10);

  num_threads = config.param<int>("global_mapping", "num_threads", 2);
}

GlobalMappingPoseGraphParams::~GlobalMappingPoseGraphParams() {}

GlobalMappingPoseGraph::GlobalMappingPoseGraph(const GlobalMappingPoseGraphParams& params) : params(params) {
  new_values.reset(new gtsam::Values);
  new_factors.reset(new gtsam::NonlinearFactorGraph);

  gtsam::ISAM2Params isam2_params;
  if (params.use_isam2_dogleg) {
    gtsam::ISAM2DoglegParams dogleg_params;
    isam2_params.setOptimizationParams(dogleg_params);
  }
  isam2_params.relinearizeSkip = params.isam2_relinearize_skip;
  isam2_params.setRelinearizeThreshold(params.isam2_relinearize_thresh);

  if (params.enable_optimization) {
    isam2.reset(new gtsam_points::ISAM2Ext(isam2_params));
  } else {
    isam2.reset(new gtsam_points::ISAM2ExtDummy(isam2_params));
  }

#ifdef GTSAM_USE_TBB
  tbb_task_arena.reset(new tbb::task_arena(params.num_threads));
#endif

  kill_switch = false;
  loop_detection_thread = std::thread([this] { loop_detection_task(); });
}

GlobalMappingPoseGraph::~GlobalMappingPoseGraph() {
  kill_switch = true;
  loop_candidates.submit_end_of_data();
  loop_detection_thread.join();
}

void GlobalMappingPoseGraph::insert_submap(const SubMap::Ptr& submap) {
  const int current = submaps.size();
  const int last = current - 1;
  insert_submap(current, submap);

  gtsam::Pose3 current_T_world_submap = gtsam::Pose3::Identity();
  gtsam::Pose3 last_T_world_submap = gtsam::Pose3::Identity();

  if (current != 0) {
    if (isam2->valueExists(X(last))) {
      last_T_world_submap = isam2->calculateEstimate<gtsam::Pose3>(X(last));
    } else {
      last_T_world_submap = new_values->at<gtsam::Pose3>(X(last));
    }

    const Eigen::Isometry3d T_origin0_endpointR0 = submaps[last]->T_origin_endpoint_R;
    const Eigen::Isometry3d T_origin1_endpointL1 = submaps[current]->T_origin_endpoint_L;
    const Eigen::Isometry3d T_endpointR0_endpointL1 = submaps[last]->odom_frames.back()->T_world_sensor().inverse() * submaps[current]->odom_frames.front()->T_world_sensor();
    const Eigen::Isometry3d T_origin0_origin1 = T_origin0_endpointR0 * T_endpointR0_endpointL1 * T_origin1_endpointL1.inverse();

    current_T_world_submap = last_T_world_submap * gtsam::Pose3(T_origin0_origin1.matrix());
  } else {
    current_T_world_submap = gtsam::Pose3(submap->T_world_origin.matrix());
  }

  new_values->insert(X(current), current_T_world_submap);
  submap->T_world_origin = Eigen::Isometry3d(current_T_world_submap.matrix());

  Callbacks::on_insert_submap(submap);

  submap->drop_frame_points();

  if (current == 0) {
    new_factors->emplace_shared<gtsam_points::LinearDampingFactor>(X(0), 6, params.init_pose_damping_scale);
  } else {
    new_factors->add(*create_odometry_factors(current));

    find_loop_candidates(current);
    new_factors->add(*collect_detected_loops());
  }

  Callbacks::on_smoother_update(*isam2, *new_factors, *new_values);
  try {
    gtsam_points::ISAM2ResultExt result;
#ifdef GTSAM_USE_TBB
    auto arena = static_cast<tbb::task_arena*>(tbb_task_arena.get());
    arena->execute([&] {
#endif
      result = isam2->update(*new_factors, *new_values);
#ifdef GTSAM_USE_TBB
    });
#endif

    Callbacks::on_smoother_update_result(*isam2, result);

  } catch (std::exception& e) {
    logger->error("an exception was caught during global map optimization!!");
    logger->error(e.what());
  }
  new_values.reset(new gtsam::Values);
  new_factors.reset(new gtsam::NonlinearFactorGraph);

  update_submaps();
  Callbacks::on_update_submaps(submaps);
}

void GlobalMappingPoseGraph::optimize() {
  if (isam2->empty()) {
    return;
  }

  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;
  new_factors.add(*collect_detected_loops());

  Callbacks::on_smoother_update(*isam2, new_factors, new_values);

  gtsam_points::ISAM2ResultExt result;
#ifdef GTSAM_USE_TBB
  auto arena = static_cast<tbb::task_arena*>(tbb_task_arena.get());
  arena->execute([&] {
#endif
    result = isam2->update(new_factors, new_values);
#ifdef GTSAM_USE_TBB
  });
#endif

  Callbacks::on_smoother_update_result(*isam2, result);

  update_submaps();
  Callbacks::on_update_submaps(submaps);
}

void GlobalMappingPoseGraph::save(const std::string& path) {
  optimize();

  boost::filesystem::create_directories(path);

  gtsam::NonlinearFactorGraph serializable_factors = isam2->getFactorsUnsafe();

  logger->info("serializing factor graph to {}/graph.bin", path);
  serializeToBinaryFile(serializable_factors, path + "/graph.bin");
  serializeToBinaryFile(isam2->calculateEstimate(), path + "/values.bin");

  std::ofstream ofs(path + "/graph.txt");
  ofs << "num_submaps: " << submaps.size() << std::endl;
  ofs << "num_all_frames: " << std::accumulate(submaps.begin(), submaps.end(), 0, [](int sum, const SubMap::ConstPtr& submap) { return sum + submap->frames.size(); }) << std::endl;

  ofs << "num_matching_cost_factors: " << 0 << std::endl;

  std::ofstream odom_lidar_ofs(path + "/odom_lidar.txt");
  std::ofstream traj_lidar_ofs(path + "/traj_lidar.txt");

  std::ofstream odom_imu_ofs(path + "/odom_imu.txt");
  std::ofstream traj_imu_ofs(path + "/traj_imu.txt");

  const auto write_tum_frame = [](std::ofstream& ofs, const double stamp, const Eigen::Isometry3d& pose) {
    const Eigen::Quaterniond quat(pose.linear());
    const Eigen::Vector3d trans(pose.translation());
    ofs << boost::format("%.9f %.6f %.6f %.6f %.6f %.6f %.6f %.6f") % stamp % trans.x() % trans.y() % trans.z() % quat.x() % quat.y() % quat.z() % quat.w() << std::endl;
  };

  for (int i = 0; i < submaps.size(); i++) {
    for (const auto& frame : submaps[i]->odom_frames) {
      write_tum_frame(odom_lidar_ofs, frame->stamp, frame->T_world_lidar);
      write_tum_frame(odom_imu_ofs, frame->stamp, frame->T_world_imu);
    }

    const Eigen::Isometry3d T_world_endpoint_L = submaps[i]->T_world_origin * submaps[i]->T_origin_endpoint_L;
    const Eigen::Isometry3d T_odom_lidar0 = submaps[i]->frames.front()->T_world_lidar;
    const Eigen::Isometry3d T_odom_imu0 = submaps[i]->frames.front()->T_world_imu;

    for (const auto& frame : submaps[i]->frames) {
      const Eigen::Isometry3d T_world_imu = T_world_endpoint_L * T_odom_imu0.inverse() * frame->T_world_imu;
      const Eigen::Isometry3d T_world_lidar = T_world_imu * frame->T_lidar_imu.inverse();

      write_tum_frame(traj_imu_ofs, frame->stamp, T_world_imu);
      write_tum_frame(traj_lidar_ofs, frame->stamp, T_world_lidar);
    }

    submaps[i]->save((boost::format("%s/%06d") % path % i).str());
  }
}

std::vector<Eigen::Vector4d> GlobalMappingPoseGraph::export_points() {
  return {};
}

void GlobalMappingPoseGraph::insert_submap(int current, const SubMap::Ptr& submap) {
  logger->debug("insert_submap id={}", submap->id);

  submap->voxelmaps.clear();

  submaps.push_back(submap);

  auto target = std::make_shared<SubMapTarget>();
  target->submap = submap;

  // Subsample points for registration
  if (params.subsample_target > 0) {
    target->subsampled = gtsam_points::random_sampling(submap->frame, static_cast<double>(params.subsample_target) / submap->frame->size(), mt);
  } else {
    if (params.subsample_rate > 0.99) {
      target->subsampled = submap->frame;
    } else {
      target->subsampled = gtsam_points::random_sampling(submap->frame, params.subsample_rate, mt);
    }
  }

  // Create nearest neighbor search
  if (params.registration_type == "GICP") {
    target->tree = std::make_shared<gtsam_points::KdTree>(submap->frame->points, submap->frame->size());
  } else if (params.registration_type == "VGICP") {
    target->voxels = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(params.vgicp_voxel_resolution);
    target->voxels->insert(*submap->frame);
  } else {
    logger->warn("unknown registration type: {}", params.registration_type);
  }

  if (current == 0) {
    target->travel_dist = 0.0;
  } else {
    const double displacement = (submaps[current - 1]->T_world_origin.translation() - submaps[current]->T_world_origin.translation()).norm();
    target->travel_dist = submap_targets.back()->travel_dist + displacement;
  }

  submap_targets.push_back(target);
}

boost::shared_ptr<gtsam::NonlinearFactorGraph> GlobalMappingPoseGraph::create_odometry_factors(int current) const {
  auto factors = gtsam::make_shared<gtsam::NonlinearFactorGraph>();
  if (current == 0) {
    return factors;
  }

  const int last = current - 1;
  const gtsam::Pose3 T_last_current = gtsam::Pose3((submaps[last]->origin_frame()->T_world_sensor().inverse() * submaps[current]->origin_frame()->T_world_sensor()).matrix());
  factors->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), T_last_current, gtsam::noiseModel::Isotropic::Sigma(6, params.odom_factor_stddev));

  return factors;
}

void GlobalMappingPoseGraph::find_loop_candidates(int current) {
  std::vector<LoopCandidate> new_candidates;
  for (int i = 0; i < submaps.size() - 1; i++) {
    // Skip if the direct distance between submaps is too far.
    const double direct_dist = (submaps[current]->T_world_origin.translation() - submaps[i]->T_world_origin.translation()).norm();
    if (direct_dist > params.max_neighbor_dist) {
      // Fast forward if the direct distance is too far.
      if (i != 0 && direct_dist > params.max_neighbor_dist * 2) {
        const int average_window = 3;
        const int left = std::max(0, i - average_window);
        const double travel_dist_avg = (submap_targets[i]->travel_dist - submap_targets[left]->travel_dist) / std::max(i - left, 1);
        const int step = 0.8 * direct_dist / std::min(travel_dist_avg, 100.0);

        i += std::min(10, step);
      }

      continue;
    }

    // Break if the travel distance is too short.
    const double travel_dist = submap_targets[current]->travel_dist - submap_targets[i]->travel_dist;
    if (travel_dist < params.min_travel_dist) {
      break;
    }

    // Add a loop candidate.
    const Eigen::Isometry3d T_target_source = submaps[i]->T_world_origin.inverse() * submaps[current]->T_world_origin;
    new_candidates.emplace_back(LoopCandidate{submap_targets[i], submap_targets[current], T_target_source});
  }

  loop_candidates.insert(new_candidates);
}

boost::shared_ptr<gtsam::NonlinearFactorGraph> GlobalMappingPoseGraph::collect_detected_loops() {
  auto factors = gtsam::make_shared<gtsam::NonlinearFactorGraph>();

  factors->add(detected_loops.get_all_and_clear());

  return factors;
}

void GlobalMappingPoseGraph::loop_detection_task() {
  std::mt19937 mt;
  std::deque<LoopCandidate> candidates_buffer;  // Local loop candidate buffer

  while (!kill_switch) {
    logger->debug("wait for loop candidates");
    auto new_candidates = loop_candidates.get_all_and_clear_wait();
    candidates_buffer.insert(candidates_buffer.end(), new_candidates.begin(), new_candidates.end());

    logger->debug("|candidates_buffer|={}", candidates_buffer.size());

    // Regulate the size of the candidate buffer.
    while (candidates_buffer.size() > params.loop_candidate_buffer_size) {
      std::shuffle(candidates_buffer.begin(), candidates_buffer.end(), mt);
      candidates_buffer.resize(params.loop_candidate_buffer_size);
    }

    // Take a subset of the candidates to evaluate.
    const int eval_count = params.loop_candidate_eval_per_thread * params.num_threads;
    std::vector<LoopCandidate> candidates;
    if (candidates_buffer.size() < eval_count) {
      candidates.assign(candidates_buffer.begin(), candidates_buffer.end());
      candidates_buffer.clear();
    } else {
      candidates.assign(candidates_buffer.begin(), candidates_buffer.begin() + eval_count);
      candidates_buffer.erase(candidates_buffer.begin(), candidates_buffer.begin() + eval_count);
    }

    std::vector<double> inlier_fractions(candidates.size(), 0.0);
    std::vector<gtsam::Pose3> T_target_source(candidates.size());

    const auto evaluate_candidate = [&](int i) {
      if (kill_switch) {
        return;
      }

      const auto candidate = candidates[i];
      const auto target = candidates[i].target;
      const auto source = candidates[i].source;

      gtsam::Values values;
      values.insert(0, gtsam::Pose3(candidates[i].init_T_target_source.matrix()));

      double error, inlier_fraction;

      if (params.registration_type == "GICP") {
        auto factor =
          gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(gtsam::Pose3(), 0, candidate.target->submap->frame, candidate.source->subsampled, candidate.target->tree);
        factor->set_max_correspondence_distance(params.gicp_max_correspondence_dist);

        gtsam::NonlinearFactorGraph graph;
        graph.add(factor);

        gtsam_points::LevenbergMarquardtExtParams lm_params;
        lm_params.setMaxIterations(10);
        values = gtsam_points::LevenbergMarquardtOptimizerExt(graph, values, lm_params).optimize();

        error = factor->error(values);
        inlier_fraction = factor->inlier_fraction();
      } else if (params.registration_type == "VGICP") {
        auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(gtsam::Pose3(), 0, candidate.target->voxels, candidate.source->subsampled);

        gtsam::NonlinearFactorGraph graph;
        graph.add(factor);

        gtsam_points::LevenbergMarquardtExtParams lm_params;
        lm_params.setMaxIterations(10);

        values = gtsam_points::LevenbergMarquardtOptimizerExt(graph, values, lm_params).optimize();

        error = factor->error(values);
        inlier_fraction = factor->inlier_fraction();
      } else {
        logger->warn("unknown registration type: {}", params.registration_type);
        return;
      }

      logger->debug("target={}, source={}, error={}, inlier_fraction={}", target->submap->id, source->submap->id, error, inlier_fraction);

      inlier_fractions[i] = inlier_fraction;
      T_target_source[i] = values.at<gtsam::Pose3>(0);
    };

    // Evaluate loop candidates in parallel.
#ifdef GTSAM_USE_TBB
    auto arena = static_cast<tbb::task_arena*>(tbb_task_arena.get());
    arena->execute([&] {
#endif
      if (gtsam_points::is_omp_default()) {
#pragma omp parallel for num_threads(params.num_threads) schedule(dynamic)
        for (int i = 0; i < candidates.size(); i++) {
          evaluate_candidate(i);
        }
      } else {
#ifdef GTSAM_POINTS_USE_TBB
        tbb::parallel_for(tbb::blocked_range<int>(0, candidates.size(), 2), [&](const tbb::blocked_range<int>& range) {
          for (int i = range.begin(); i < range.end(); i++) {
            evaluate_candidate(i);
          }
        });
#else
      std::cerr << "error : TBB is not enabled" << std::endl;
      abort();
#endif
      }

#ifdef GTSAM_USE_TBB
    });
#endif

    // Check the matching results.
    std::vector<gtsam::NonlinearFactor::shared_ptr> factors;
    for (int i = 0; i < candidates.size(); i++) {
      // Check if the inlier fraction (overlap with target) is large enough.
      if (inlier_fractions[i] < params.min_inliear_fraction) {
        continue;
      }

      // Create factor.
      gtsam::SharedNoiseModel noise_model = gtsam::noiseModel::Isotropic::Sigma(6, params.loop_factor_stddev);
      noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(params.loop_factor_robust_width), noise_model);
      factors.emplace_back(
        gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(candidates[i].target->submap->id), X(candidates[i].source->submap->id), T_target_source[i], noise_model));
    }

    detected_loops.insert(factors);
  }
}

void GlobalMappingPoseGraph::update_submaps() {
  for (int i = 0; i < submaps.size(); i++) {
    submaps[i]->T_world_origin = Eigen::Isometry3d(isam2->calculateEstimate<gtsam::Pose3>(X(i)).matrix());
  }
}

}  // namespace glim