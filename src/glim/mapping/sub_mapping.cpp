#include <glim/mapping/sub_mapping.hpp>

#include <filesystem>
#include <spdlog/spdlog.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam_points/config.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/point_cloud_gpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/cuda/cuda_stream.hpp>
#include <gtsam_points/cuda/stream_temp_buffer_roundrobin.hpp>

#include <glim/util/config.hpp>
#include <glim/util/convert_to_string.hpp>
#include <glim/common/imu_integration.hpp>
#include <glim/common/cloud_deskewing.hpp>
#include <glim/common/cloud_covariance_estimation.hpp>
#include <glim/mapping/callbacks.hpp>

#ifdef GTSAM_USE_TBB
#include <tbb/task_arena.h>
#endif

namespace glim {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

using Callbacks = SubMappingCallbacks;

SubMappingParams::SubMappingParams() {
  Config config(GlobalConfig::get_config_path("config_sub_mapping"));

  enable_imu = config.param<bool>("sub_mapping", "enable_imu", true);
  enable_optimization = config.param<bool>("sub_mapping", "enable_optimization", true);

  max_num_keyframes = config.param<int>("sub_mapping", "max_num_keyframes", 15);

  keyframe_update_strategy = config.param<std::string>("sub_mapping", "keyframe_update_strategy", "OVERLAP");
  keyframe_update_min_points = config.param<int>("sub_mapping", "keyframe_update_min_points", 500);
  keyframe_update_interval_rot = config.param<double>("sub_mapping", "keyframe_update_interval_rot", 3.15);
  keyframe_update_interval_trans = config.param<double>("sub_mapping", "keyframe_update_interval_trans", 1.0);
  max_keyframe_overlap = config.param<double>("sub_mapping", "max_keyframe_overlap", 0.8);

  create_between_factors = config.param<bool>("sub_mapping", "create_between_factors", true);
  between_registration_type = config.param<std::string>("sub_mapping", "between_registration_type", "GICP");

  registration_error_factor_type = config.param<std::string>("sub_mapping", "registration_error_factor_type", "VGICP");
  keyframe_randomsampling_rate = config.param<double>("sub_mapping", "keyframe_randomsampling_rate", 0.1);
  keyframe_voxel_resolution = config.param<double>("sub_mapping", "keyframe_voxel_resolution", 0.5);
  keyframe_voxelmap_levels = config.param<int>("sub_mapping", "keyframe_voxelmap_levels", 3);
  keyframe_voxelmap_scaling_factor = config.param<double>("sub_mapping", "keyframe_voxelmap_scaling_factor", 2.0);

  submap_downsample_resolution = config.param<double>("sub_mapping", "submap_downsample_resolution", 0.25);
  submap_voxel_resolution = config.param<double>("sub_mapping", "submap_voxel_resolution", 0.5);
  submap_target_num_points = config.param<int>("sub_mapping", "submap_target_num_points", -1);

  enable_gpu = false;
  if (registration_error_factor_type.find("GPU") != std::string::npos) {
    enable_gpu = true;
  }
}

SubMappingParams::~SubMappingParams() {}

SubMapping::SubMapping(const SubMappingParams& params) : params(params) {
  submap_count = 0;
  imu_integration.reset(new IMUIntegration);
  deskewing.reset(new CloudDeskewing);
  covariance_estimation.reset(new CloudCovarianceEstimation);

  values.reset(new gtsam::Values);
  graph.reset(new gtsam::NonlinearFactorGraph);

#ifdef GTSAM_POINTS_USE_CUDA
  stream = std::make_shared<gtsam_points::CUDAStream>();
  stream_buffer_roundrobin = std::make_shared<gtsam_points::StreamTempBufferRoundRobin>(8);
#endif

#ifdef GTSAM_USE_TBB
  tbb_task_arena = std::make_shared<tbb::task_arena>(1);
#endif
}

SubMapping::~SubMapping() {}

void SubMapping::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Callbacks::on_insert_imu(stamp, linear_acc, angular_vel);
  if (params.enable_imu) {
    imu_integration->insert_imu(stamp, linear_acc, angular_vel);
  }
}

void SubMapping::insert_frame(const EstimationFrame::ConstPtr& odom_frame_) {
  logger->trace("insert_frame frame_id={} stamp={}", odom_frame_->id, odom_frame_->stamp);
  Callbacks::on_insert_frame(odom_frame_);

  delayed_input_queue.emplace_back(odom_frame_);
  if (delayed_input_queue.size() < 2) {
    return;
  }

  EstimationFrame::Ptr odom_frame = delayed_input_queue.front()->clone();
  delayed_input_queue.pop_front();
  EstimationFrame::ConstPtr next_frame = delayed_input_queue.front();

  if (params.enable_imu) {
    logger->debug("smoothing trajectory");
    // Smoothing IMU-based pose estimation
    gtsam::NavState nav_world_imu(gtsam::Pose3(odom_frame->T_world_imu.matrix()), odom_frame->v_world_imu);
    gtsam::imuBias::ConstantBias imu_bias(odom_frame->imu_bias);

    std::vector<double> imu_stamps;
    std::vector<Eigen::Isometry3d> imu_poses;
    imu_integration->integrate_imu(odom_frame->stamp, next_frame->stamp, nav_world_imu, imu_bias, imu_stamps, imu_poses);

    gtsam::Values values;
    for (int i = 0; i < imu_stamps.size(); i++) {
      values.insert(X(i), gtsam::Pose3(imu_poses[i].matrix()));
    }

    gtsam::NonlinearFactorGraph graph;
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), gtsam::Pose3(odom_frame->T_world_imu.matrix()), gtsam::noiseModel::Isotropic::Sigma(6, 1e-5));
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(imu_stamps.size() - 1), gtsam::Pose3(next_frame->T_world_imu.matrix()), gtsam::noiseModel::Isotropic::Sigma(6, 1e-5));
    for (int i = 1; i < imu_stamps.size(); i++) {
      const double dt = (imu_stamps[i] - imu_stamps[i - 1]) / (next_frame->stamp - odom_frame->stamp);
      const Eigen::Isometry3d T_last_current = imu_poses[i - 1].inverse() * imu_poses[i];
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(i - 1), X(i), gtsam::Pose3(T_last_current.matrix()), gtsam::noiseModel::Isotropic::Sigma(6, dt + 1e-2));
    }

    gtsam::LevenbergMarquardtParams lm_params;
    lm_params.setAbsoluteErrorTol(1e-6);
    lm_params.setRelativeErrorTol(1e-6);
    lm_params.setMaxIterations(5);

#ifdef GTSAM_USE_TBB
    auto arena = static_cast<tbb::task_arena*>(this->tbb_task_arena.get());
    arena->execute([&] {
#endif
      values = gtsam::LevenbergMarquardtOptimizer(graph, values, lm_params).optimize();
#ifdef GTSAM_USE_TBB
    });
#endif

    odom_frame->imu_rate_trajectory.resize(8, imu_stamps.size());
    for (int i = 0; i < imu_stamps.size(); i++) {
      const Eigen::Vector3d trans(imu_poses[i].translation());
      const Eigen::Quaterniond quat(imu_poses[i].linear());
      odom_frame->imu_rate_trajectory.col(i) << imu_stamps[i], trans, quat.x(), quat.y(), quat.z(), quat.w();
    }
  }

#ifdef GTSAM_POINTS_USE_CUDA
  if (params.enable_gpu && !odom_frame->frame->points_gpu) {
    if (params.enable_gpu) {
      auto stream = std::static_pointer_cast<gtsam_points::CUDAStream>(this->stream);
      auto frame_gpu = gtsam_points::PointCloudGPU::clone(*odom_frame->frame, *stream);
      odom_frame->frame = frame_gpu;
    }
  }
#endif

  const int current = odom_frames.size();
  const int last = current - 1;
  odom_frames.push_back(odom_frame);
  values->insert(X(current), gtsam::Pose3(odom_frame->T_world_sensor().matrix()));

  if (params.enable_imu && odom_frame->frame_id != FrameID::IMU) {
    logger->warn("odom frames are not estimated in the IMU frame while sub_mapping requires IMU estimation");
  }

  // Fix the first frame
  if (current == 0) {
    logger->debug("first frame in submap");
    graph->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), values->at<gtsam::Pose3>(X(0)), gtsam::noiseModel::Isotropic::Precision(6, 1e8));
  }
  // Create a relative pose factor between consecutive frames
  else if (params.create_between_factors) {
    logger->debug("create between factors");
    const Eigen::Isometry3d delta = odom_frames[last]->T_world_sensor().inverse() * odom_frame->T_world_sensor();

    if (params.between_registration_type == "GICP") {
      const auto& last_frame = odom_frames[last]->frame;
      const auto& current_frame = odom_frames[current]->frame;

      gtsam::noiseModel::Base::shared_ptr noise_model;
      if (last_frame->size() < 500 || current_frame->size() < 500) {
        logger->warn("use an identity covariance because either of last or current frames have too few points (last={} current={})", last_frame->size(), current_frame->size());
        noise_model = gtsam::noiseModel::Isotropic::Precision(6, 1e3);
      } else {
        auto factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(X(last), X(current), last_frame, current_frame);
        auto linearized = factor->linearize(*values);
        // graph->emplace_shared<gtsam::LinearContainerFactor>(linearized, *values);

        auto H = linearized->hessianBlockDiagonal()[X(current)];
        noise_model = gtsam::noiseModel::Gaussian::Information(H);
      }

      graph->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), gtsam::Pose3(delta.matrix()), noise_model);
    } else if (params.between_registration_type == "NONE") {
      graph->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), gtsam::Pose3(delta.matrix()), gtsam::noiseModel::Isotropic::Precision(6, 1e3));
    } else {
      logger->warn("unknown between registration type ({})", params.between_registration_type);
    }
  }

  // Create an IMU preintegration factor
  if (params.enable_imu) {
    logger->debug("create IMU factor");
    const gtsam::imuBias::ConstantBias imu_bias(odom_frame->imu_bias);

    values->insert(V(current), odom_frame->v_world_imu);
    values->insert(B(current), imu_bias);

    graph->emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(current), odom_frame->v_world_imu, gtsam::noiseModel::Isotropic::Precision(3, 1e3));
    graph->emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(current), imu_bias, gtsam::noiseModel::Isotropic::Precision(6, 1e6));

    if (current != 0) {
      int num_integrated = 0;
      const int imu_read_cursor = imu_integration->integrate_imu(odom_frames[last]->stamp, odom_frames[current]->stamp, imu_bias, &num_integrated);
      imu_integration->erase_imu_data(imu_read_cursor);

      graph
        ->emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(B(last), B(current), gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Isotropic::Precision(6, 1e6));
      if (num_integrated >= 2) {
        graph->emplace_shared<gtsam::ImuFactor>(X(last), V(last), X(current), V(current), B(last), imu_integration->integrated_measurements());
      } else {
        logger->warn("insufficient IMU data between LiDAR frames!! (sub_mapping)");
        graph->emplace_shared<gtsam::BetweenFactor<gtsam::Vector3>>(V(last), V(current), gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Precision(3, 1.0));
      }
    }
  }

  bool insert_as_keyframe = keyframes.empty();
  if (!insert_as_keyframe && odom_frame->frame && odom_frame->frame->size() > params.keyframe_update_min_points) {
    // Overlap-based keyframe update
    if (params.keyframe_update_strategy == "OVERLAP") {
      if (keyframes.back()->voxelmaps.empty() || odom_frame->frame->size() < 10) {
        logger->warn("voxelmap or odom_frame is empty!! (voxelmap={} odom_frame={})", keyframes.back()->voxelmaps.size(), odom_frame->frame->size());
      } else {
        const double overlap =
          gtsam_points::overlap_auto(keyframes.back()->voxelmaps.back(), odom_frame->frame, keyframes.back()->T_world_sensor().inverse() * odom_frame->T_world_sensor());
        insert_as_keyframe = overlap < params.max_keyframe_overlap;
      }
    }
    // Displacement-based keyframe update
    else if (params.keyframe_update_strategy == "DISPLACEMENT") {
      const Eigen::Isometry3d delta_from_keyframe = keyframes.back()->T_world_sensor().inverse() * odom_frame->T_world_sensor();
      const double delta_trans = delta_from_keyframe.translation().norm();
      const double delta_angle = Eigen::AngleAxisd(delta_from_keyframe.linear()).angle();

      insert_as_keyframe = delta_trans > params.keyframe_update_interval_trans || delta_angle > params.keyframe_update_interval_rot;
    } else {
      logger->warn("unknown keyframe update strategy ({})", params.keyframe_update_strategy);
    }
  }

  // Create a new keyframe
  if (insert_as_keyframe) {
    logger->debug("insert frame as keyframe");
    insert_keyframe(current, odom_frame);
    Callbacks::on_new_keyframe(current, keyframes.back());

    // Create registration error factors (fully connected)
    for (int i = 0; i < keyframes.size() - 1; i++) {
      if (keyframes[i]->frame->size() == 0 || keyframes.back()->frame->size() == 0) {
        logger->warn(
          "skip creation of registration error factors because keyframe has no points (keyframe[i]={}, keyframe[-1]={})",
          keyframes[i]->frame->size(),
          keyframes.back()->frame->size());
      }

      if (params.registration_error_factor_type == "VGICP") {
        for (const auto& voxelmap : keyframes[i]->voxelmaps) {
          if (!voxelmap) {
            logger->warn("voxelmap is empty!");
            continue;
          }

          graph->emplace_shared<gtsam_points::IntegratedVGICPFactor>(X(keyframe_indices[i]), X(current), voxelmap, keyframes.back()->frame);
        }
      }
#ifdef GTSAM_POINTS_USE_CUDA
      else if (params.registration_error_factor_type == "VGICP_GPU") {
        auto roundrobin = std::static_pointer_cast<gtsam_points::StreamTempBufferRoundRobin>(stream_buffer_roundrobin);
        auto stream_buffer = roundrobin->get_stream_buffer();
        const auto& stream = stream_buffer.first;
        const auto& buffer = stream_buffer.second;

        for (const auto& voxelmap : keyframes[i]->voxelmaps) {
          if (!voxelmap) {
            logger->warn("voxelmap is empty!");
            continue;
          }

          auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactorGPU>(X(keyframe_indices[i]), X(current), voxelmap, keyframes.back()->frame, stream, buffer);
          graph->add(factor);
        }
      }
#endif
      else {
        logger->warn("unknown registration error factor type ({})", params.registration_error_factor_type);
      }
    }
  }

  if (odom_frames.size() >= 2) {
    // Drop unnecessary points data
    // The last frame may be required to compute the relative pose factor
    odom_frames[odom_frames.size() - 2] = odom_frames[odom_frames.size() - 2]->clone_wo_points();
  }

  auto new_submap = create_submap();

  if (new_submap) {
    new_submap->id = submap_count++;
    submap_queue.push_back(new_submap);
    Callbacks::on_new_submap(new_submap);

    odom_frames.clear();
    keyframes.clear();
    keyframe_indices.clear();
    values.reset(new gtsam::Values);
    graph.reset(new gtsam::NonlinearFactorGraph);
  }
}

void SubMapping::insert_keyframe(const int current, const EstimationFrame::ConstPtr& odom_frame) {
  gtsam_points::PointCloud::ConstPtr deskewed_frame = odom_frame->frame;

  // Re-perform deskewing with smoothed IMU poses
  if (params.enable_imu && odom_frame->raw_frame && odom_frame->imu_rate_trajectory.cols() >= 2) {
    if (std::abs(odom_frame->stamp - odom_frame->imu_rate_trajectory(0, 0)) > 1e-3) {
      logger->warn("inconsistent frame stamp and imu_rate stamp!! (odom_frame={} imu_rate_trajectory={})", odom_frame->stamp, odom_frame->imu_rate_trajectory(0, 0));
    }
    if (odom_frame->raw_frame->scan_end_time > odom_frame->imu_rate_trajectory.rightCols<1>()[0] + 1e-3) {
      logger->warn(
        "imu_rate stamp does not cover the scan duration range!! (imu_rate_end={} scan_end={})",
        odom_frame->imu_rate_trajectory.rightCols<1>()[0],
        odom_frame->raw_frame->scan_end_time);
    }

    std::vector<double> imu_pred_times(odom_frame->imu_rate_trajectory.cols());
    std::vector<Eigen::Isometry3d> imu_pred_poses(odom_frame->imu_rate_trajectory.cols());
    for (int i = 0; i < odom_frame->imu_rate_trajectory.cols(); i++) {
      const Eigen::Matrix<double, 8, 1> imu = odom_frame->imu_rate_trajectory.col(i).transpose();
      imu_pred_times[i] = imu[0];
      imu_pred_poses[i].setIdentity();
      imu_pred_poses[i].translation() << imu[1], imu[2], imu[3];
      imu_pred_poses[i].linear() = Eigen::Quaterniond(imu[7], imu[4], imu[5], imu[6]).toRotationMatrix();
    }

    auto deskewed =
      deskewing
        ->deskew(odom_frame->T_lidar_imu.inverse(), imu_pred_times, imu_pred_poses, odom_frame->raw_frame->stamp, odom_frame->raw_frame->times, odom_frame->raw_frame->points);

    auto frame = std::make_shared<gtsam_points::PointCloudCPU>(deskewed);
    for (int i = 0; i < frame->size(); i++) {
      frame->points[i] = odom_frame->T_lidar_imu.inverse() * frame->points[i];
    }
    frame->add_covs(covariance_estimation->estimate(frame->points_storage, odom_frame->raw_frame->neighbors));

    deskewed_frame = frame;
  }

  // Random sampling for registration error factors
  gtsam_points::PointCloud::Ptr subsampled_frame = gtsam_points::random_sampling(deskewed_frame, params.keyframe_randomsampling_rate, mt);

  EstimationFrame::Ptr keyframe(new EstimationFrame);
  *keyframe = *odom_frame;

  if (params.enable_gpu) {
#ifdef GTSAM_POINTS_USE_CUDA
    auto stream = std::static_pointer_cast<gtsam_points::CUDAStream>(this->stream);
    keyframe->frame = gtsam_points::PointCloudGPU::clone(*subsampled_frame, *stream);
    keyframe->voxelmaps.clear();

    for (int i = 0; i < params.keyframe_voxelmap_levels; i++) {
      const double resolution = params.keyframe_voxel_resolution * std::pow(params.keyframe_voxelmap_scaling_factor, i);
      auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(resolution, 8192 * 2, 10, 1e-3, *stream);
      voxelmap->insert(*keyframe->frame);
      keyframe->voxelmaps.push_back(voxelmap);
    }
#else
    logger->warn("GPU is enabled for sub_mapping but gtsam_points was built without CUDA!!");
#endif
  } else {
    keyframe->voxelmaps.clear();
    for (int i = 0; i < params.keyframe_voxelmap_levels; i++) {
      const double resolution = params.keyframe_voxel_resolution * std::pow(params.keyframe_voxelmap_scaling_factor, i);
      auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
      voxelmap->insert(*keyframe->frame);
      keyframe->voxelmaps.push_back(voxelmap);
    }

    keyframe->frame = subsampled_frame;
  }

  keyframes.push_back(keyframe);
  keyframe_indices.push_back(current);
}

SubMap::Ptr SubMapping::create_submap(bool force_create) const {
  logger->debug("|keyframes|={}", keyframes.size());
  if (keyframes.size() < params.max_num_keyframes && !force_create) {
    return nullptr;
  }

  logger->debug("create_submap");
  // Optimization
  Callbacks::on_optimize_submap(*graph, *values);
  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setMaxIterations(20);
  if (Callbacks::on_optimization_status) {
    lm_params.callback = [](const gtsam_points::LevenbergMarquardtOptimizationStatus& status, const gtsam::Values& values) { Callbacks::on_optimization_status(status, values); };
  }
  gtsam_points::LevenbergMarquardtOptimizerExt optimizer(*graph, *values, lm_params);
  if (params.enable_optimization) {
    try {
#ifdef GTSAM_USE_TBB
      auto arena = static_cast<tbb::task_arena*>(this->tbb_task_arena.get());
      arena->execute([&] {
#endif
        gtsam::Values optimized = optimizer.optimize();
        *values = optimized;

#ifdef GTSAM_USE_TBB
      });
#endif
    } catch (std::exception& e) {
      logger->error("an exception was caught during sub map optimization");
      logger->error(e.what());
    }
  }

  // Create a submap by merging optimized frames
  SubMap::Ptr submap(new SubMap);
  submap->id = 0;

  const int center = odom_frames.size() / 2;
  submap->T_world_origin = Eigen::Isometry3d(values->at<gtsam::Pose3>(X(center)).matrix());
  submap->T_origin_endpoint_L = submap->T_world_origin.inverse() * Eigen::Isometry3d(values->at<gtsam::Pose3>(X(0)).matrix());
  submap->T_origin_endpoint_R = submap->T_world_origin.inverse() * Eigen::Isometry3d(values->at<gtsam::Pose3>(X(odom_frames.size() - 1)).matrix());

  submap->odom_frames = odom_frames;
  submap->frames.resize(odom_frames.size());
  for (int i = 0; i < odom_frames.size(); i++) {
    EstimationFrame::Ptr frame(new EstimationFrame);
    *frame = *odom_frames[i];

    const Eigen::Isometry3d T_world_sensor(values->at<gtsam::Pose3>(X(i)).matrix());
    frame->set_T_world_sensor(odom_frames[i]->frame_id, T_world_sensor);

    if (params.enable_imu) {
      frame->v_world_imu = values->at<gtsam::Vector3>(V(i));
      frame->imu_bias = values->at<gtsam::imuBias::ConstantBias>(B(i)).vector();
    }

    submap->frames[i] = frame;
  }

  logger->debug("merge frames");
  std::vector<gtsam_points::PointCloud::ConstPtr> keyframes_to_merge(keyframes.size());
  std::vector<Eigen::Isometry3d> poses_to_merge(keyframes.size());
  for (int i = 0; i < keyframes.size(); i++) {
    keyframes_to_merge[i] = keyframes[i]->frame;
    poses_to_merge[i] = submap->T_world_origin.inverse() * Eigen::Isometry3d(values->at<gtsam::Pose3>(X(keyframe_indices[i])).matrix());
  }

  // TODO: improve merging process
#ifdef GTSAM_POINTS_USE_CUDA
  if (params.enable_gpu) {
    // submap->frame = gtsam_points::merge_frames_gpu(poses_to_merge, keyframes_to_merge, submap_downsample_resolution);
  }
#endif

  if (submap->frame == nullptr) {
    submap->frame = gtsam_points::merge_frames_auto(poses_to_merge, keyframes_to_merge, params.submap_downsample_resolution);
  }
  logger->debug("|merged_submap|={}", submap->frame->size());

  if (params.submap_target_num_points > 0 && submap->frame->size() > params.submap_target_num_points) {
    std::mt19937 mt(submap_count * 643145 + submap->frame->size() * 4312);  // Just a random-like seed
    submap->frame = gtsam_points::random_sampling(submap->frame, static_cast<double>(params.submap_target_num_points) / submap->frame->size(), mt);
    logger->debug("|subsampled_submap|={}", submap->frame->size());
  }

  return submap;
}

std::vector<SubMap::Ptr> SubMapping::get_submaps() {
  std::vector<SubMap::Ptr> submaps;
  submap_queue.swap(submaps);
  return submaps;
}

std::vector<SubMap::Ptr> SubMapping::submit_end_of_sequence() {
  std::vector<SubMap::Ptr> submaps;
  if (!odom_frames.empty()) {
    auto new_submap = create_submap(true);

    if (new_submap) {
      new_submap->id = submap_count++;
      submaps.push_back(new_submap);
    }
  }

  return submaps;
}

}  // namespace glim
