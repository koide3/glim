#include "pyglim.hpp"

#include <gtsam_points/types/point_cloud.hpp>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/odometry/odometry_estimation_base.hpp>
#include <glim/odometry/odometry_estimation_cpu.hpp>
#include <glim/odometry/odometry_estimation_ct.hpp>
#ifdef PYGLIM_USE_CUDA
#include <glim/odometry/odometry_estimation_gpu.hpp>
#endif
#include <glim/odometry/async_odometry_estimation.hpp>

void define_odometry(py::module_& m) {
  // glim::FrameID
  py::enum_<glim::FrameID>(m, "FrameID", "Coordinate frame ID")
    .value("WORLD", glim::FrameID::WORLD)
    .value("LIDAR", glim::FrameID::LIDAR)
    .value("IMU", glim::FrameID::IMU)
    .export_values();

  // glim::EstimationFrame
  auto est_frame = py::class_<glim::EstimationFrame, std::shared_ptr<glim::EstimationFrame>>(m, "EstimationFrame", "Odometry estimation frame")
    .def("__repr__", [](const glim::EstimationFrame& f) { return "<pyglim.EstimationFrame id=" + std::to_string(f.id) + " stamp=" + std::to_string(f.stamp) + ">"; })
    .def_readonly("id", &glim::EstimationFrame::id, "Frame ID")
    .def_readonly("stamp", &glim::EstimationFrame::stamp, "Timestamp")
    .def_property_readonly("v_world_imu", [](const glim::EstimationFrame& f) -> Eigen::Vector3d { return f.v_world_imu; }, "IMU velocity in the world frame")
    .def_property_readonly("imu_bias", [](const glim::EstimationFrame& f) -> Eigen::Matrix<double, 6, 1> { return f.imu_bias; }, "IMU bias [acc, gyro]")
    .def_readonly("cov_computed", &glim::EstimationFrame::cov_computed, "Whether the covariances have been computed")
    .def_property_readonly("pose_cov", [](const glim::EstimationFrame& f) -> Eigen::Matrix<double, 6, 6> { return f.pose_cov; }, "Pose covariance")
    .def_property_readonly("vel_cov", [](const glim::EstimationFrame& f) -> Eigen::Matrix3d { return f.vel_cov; }, "Velocity covariance")
    .def_property_readonly("bias_cov", [](const glim::EstimationFrame& f) -> Eigen::Matrix<double, 6, 6> { return f.bias_cov; }, "IMU bias covariance")
    .def_readonly("frame_id", &glim::EstimationFrame::frame_id, "Coordinate frame of the deskewed points")
    .def(
      "T_world_sensor",
      [](const glim::EstimationFrame& f) -> Eigen::Matrix4d { return f.T_world_sensor().matrix(); },
      "Sensor pose according to frame_id (4x4 matrix)")
    .def_property_readonly(
      "num_points",
      [](const glim::EstimationFrame& f) -> size_t { return f.frame ? f.frame->size() : 0; },
      "Number of deskewed points")
    .def_property_readonly(
      "points",
      [](const glim::EstimationFrame& f) {
        if (!f.frame || !f.frame->points) {
          return convert_points(nullptr, 0);
        }
        return convert_points(f.frame->points, f.frame->size());
      },
      "Deskewed points for state estimation [N, 3] (in frame_id coordinates)")
    .def_property_readonly(
      "imu_rate_trajectory",
      [](const glim::EstimationFrame& f) -> Eigen::MatrixXd { return f.imu_rate_trajectory.transpose(); },
      "IMU-rate trajectory [N, 8] (t, x, y, z, qx, qy, qz, qw)")
    .def_property_readonly(
      "raw_frame",
      [](const glim::EstimationFrame& f) { return unconst(f.raw_frame); },
      "Raw input point cloud (LiDAR frame)");
  def_isometry_property(est_frame, "T_lidar_imu", &glim::EstimationFrame::T_lidar_imu, "LiDAR-IMU transformation (4x4 matrix)");
  def_isometry_property(est_frame, "T_world_lidar", &glim::EstimationFrame::T_world_lidar, "LiDAR pose in the world frame (4x4 matrix)");
  def_isometry_property(est_frame, "T_world_imu", &glim::EstimationFrame::T_world_imu, "IMU pose in the world frame (4x4 matrix)");

  // glim::OdometryEstimationIMUParams
  auto imu_params = py::class_<glim::OdometryEstimationIMUParams>(m, "OdometryEstimationIMUParams", "Parameters for IMU-coupled odometry estimation")
    .def_readwrite("fix_imu_bias", &glim::OdometryEstimationIMUParams::fix_imu_bias)
    .def_readwrite("imu_bias_noise", &glim::OdometryEstimationIMUParams::imu_bias_noise)
    .def_property(
      "imu_bias",
      [](const glim::OdometryEstimationIMUParams& p) -> Eigen::Matrix<double, 6, 1> { return p.imu_bias; },
      [](glim::OdometryEstimationIMUParams& p, const Eigen::Matrix<double, 6, 1>& bias) { p.imu_bias = bias; })
    .def_readwrite("initialization_mode", &glim::OdometryEstimationIMUParams::initialization_mode)
    .def_readwrite("estimate_init_state", &glim::OdometryEstimationIMUParams::estimate_init_state)
    .def_property(
      "init_v_world_imu",
      [](const glim::OdometryEstimationIMUParams& p) -> Eigen::Vector3d { return p.init_v_world_imu; },
      [](glim::OdometryEstimationIMUParams& p, const Eigen::Vector3d& v) { p.init_v_world_imu = v; })
    .def_readwrite("init_pose_damping_scale", &glim::OdometryEstimationIMUParams::init_pose_damping_scale)
    .def_readwrite("smoother_lag", &glim::OdometryEstimationIMUParams::smoother_lag)
    .def_readwrite("use_isam2_dogleg", &glim::OdometryEstimationIMUParams::use_isam2_dogleg)
    .def_readwrite("isam2_relinearize_skip", &glim::OdometryEstimationIMUParams::isam2_relinearize_skip)
    .def_readwrite("isam2_relinearize_thresh", &glim::OdometryEstimationIMUParams::isam2_relinearize_thresh)
    .def_readwrite("compute_covs", &glim::OdometryEstimationIMUParams::compute_covs)
    .def_readwrite("validate_imu", &glim::OdometryEstimationIMUParams::validate_imu)
    .def_readwrite("save_imu_rate_trajectory", &glim::OdometryEstimationIMUParams::save_imu_rate_trajectory)
    .def_readwrite("num_threads", &glim::OdometryEstimationIMUParams::num_threads)
    .def_readwrite("num_smoother_update_threads", &glim::OdometryEstimationIMUParams::num_smoother_update_threads);
  def_isometry_property(imu_params, "T_lidar_imu", &glim::OdometryEstimationIMUParams::T_lidar_imu, "LiDAR-IMU transformation (4x4 matrix)");
  def_isometry_property(imu_params, "init_T_world_imu", &glim::OdometryEstimationIMUParams::init_T_world_imu, "Initial IMU pose (4x4 matrix)");

  // glim::OdometryEstimationCPUParams
  py::class_<glim::OdometryEstimationCPUParams, glim::OdometryEstimationIMUParams>(m, "OdometryEstimationCPUParams", "Parameters for OdometryEstimationCPU")
    .def(py::init<>(), "Load parameters from the global config")
    .def_readwrite("registration_type", &glim::OdometryEstimationCPUParams::registration_type, "Registration type (GICP or VGICP)")
    .def_readwrite("max_iterations", &glim::OdometryEstimationCPUParams::max_iterations)
    .def_readwrite("lru_thresh", &glim::OdometryEstimationCPUParams::lru_thresh)
    .def_readwrite("target_downsampling_rate", &glim::OdometryEstimationCPUParams::target_downsampling_rate)
    .def_readwrite("ivox_resolution", &glim::OdometryEstimationCPUParams::ivox_resolution)
    .def_readwrite("ivox_min_dist", &glim::OdometryEstimationCPUParams::ivox_min_dist)
    .def_readwrite("vgicp_resolution", &glim::OdometryEstimationCPUParams::vgicp_resolution)
    .def_readwrite("vgicp_voxelmap_levels", &glim::OdometryEstimationCPUParams::vgicp_voxelmap_levels)
    .def_readwrite("vgicp_voxelmap_scaling_factor", &glim::OdometryEstimationCPUParams::vgicp_voxelmap_scaling_factor);

  // glim::OdometryEstimationCTParams
  py::class_<glim::OdometryEstimationCTParams>(m, "OdometryEstimationCTParams", "Parameters for OdometryEstimationCT")
    .def(py::init<>(), "Load parameters from the global config")
    .def_readwrite("num_threads", &glim::OdometryEstimationCTParams::num_threads)
    .def_readwrite("ivox_resolution", &glim::OdometryEstimationCTParams::ivox_resolution)
    .def_readwrite("ivox_min_points_dist", &glim::OdometryEstimationCTParams::ivox_min_points_dist)
    .def_readwrite("ivox_lru_thresh", &glim::OdometryEstimationCTParams::ivox_lru_thresh)
    .def_readwrite("max_correspondence_distance", &glim::OdometryEstimationCTParams::max_correspondence_distance)
    .def_readwrite("location_consistency_inf_scale", &glim::OdometryEstimationCTParams::location_consistency_inf_scale)
    .def_readwrite("constant_velocity_inf_scale", &glim::OdometryEstimationCTParams::constant_velocity_inf_scale)
    .def_readwrite("lm_max_iterations", &glim::OdometryEstimationCTParams::lm_max_iterations)
    .def_readwrite("smoother_lag", &glim::OdometryEstimationCTParams::smoother_lag)
    .def_readwrite("use_isam2_dogleg", &glim::OdometryEstimationCTParams::use_isam2_dogleg)
    .def_readwrite("isam2_relinearize_skip", &glim::OdometryEstimationCTParams::isam2_relinearize_skip)
    .def_readwrite("isam2_relinearize_thresh", &glim::OdometryEstimationCTParams::isam2_relinearize_thresh)
    .def_readwrite("compute_covs", &glim::OdometryEstimationCTParams::compute_covs);

#ifdef PYGLIM_USE_CUDA
  // glim::OdometryEstimationGPUParams
  auto gpu_params = py::class_<glim::OdometryEstimationGPUParams, glim::OdometryEstimationIMUParams>(m, "OdometryEstimationGPUParams", "Parameters for OdometryEstimationGPU")
    .def(py::init<>(), "Load parameters from the global config")
    .def_readwrite("voxel_resolution", &glim::OdometryEstimationGPUParams::voxel_resolution)
    .def_readwrite("voxel_resolution_max", &glim::OdometryEstimationGPUParams::voxel_resolution_max)
    .def_readwrite("voxel_resolution_dmin", &glim::OdometryEstimationGPUParams::voxel_resolution_dmin)
    .def_readwrite("voxel_resolution_dmax", &glim::OdometryEstimationGPUParams::voxel_resolution_dmax)
    .def_readwrite("voxelmap_levels", &glim::OdometryEstimationGPUParams::voxelmap_levels)
    .def_readwrite("voxelmap_scaling_factor", &glim::OdometryEstimationGPUParams::voxelmap_scaling_factor)
    .def_readwrite("max_num_keyframes", &glim::OdometryEstimationGPUParams::max_num_keyframes)
    .def_readwrite("full_connection_window_size", &glim::OdometryEstimationGPUParams::full_connection_window_size)
    .def_readwrite("keyframe_strategy", &glim::OdometryEstimationGPUParams::keyframe_strategy)
    .def_readwrite("keyframe_min_overlap", &glim::OdometryEstimationGPUParams::keyframe_min_overlap)
    .def_readwrite("keyframe_max_overlap", &glim::OdometryEstimationGPUParams::keyframe_max_overlap)
    .def_readwrite("keyframe_delta_trans", &glim::OdometryEstimationGPUParams::keyframe_delta_trans)
    .def_readwrite("keyframe_delta_rot", &glim::OdometryEstimationGPUParams::keyframe_delta_rot)
    .def_readwrite("keyframe_entropy_thresh", &glim::OdometryEstimationGPUParams::keyframe_entropy_thresh);

  py::enum_<glim::OdometryEstimationGPUParams::KeyframeUpdateStrategy>(gpu_params, "KeyframeUpdateStrategy")
    .value("OVERLAP", glim::OdometryEstimationGPUParams::KeyframeUpdateStrategy::OVERLAP)
    .value("DISPLACEMENT", glim::OdometryEstimationGPUParams::KeyframeUpdateStrategy::DISPLACEMENT)
    .value("ENTROPY", glim::OdometryEstimationGPUParams::KeyframeUpdateStrategy::ENTROPY)
    .export_values();
#endif

  // glim::OdometryEstimationBase
  py::class_<glim::OdometryEstimationBase, std::shared_ptr<glim::OdometryEstimationBase>>(m, "OdometryEstimationBase", "Odometry estimation base class")
    .def_static("load_module", &glim::OdometryEstimationBase::load_module, py::arg("so_name"), "Load an odometry estimation module from a shared library (e.g., libodometry_estimation_cpu.so)")
    .def("requires_imu", &glim::OdometryEstimationBase::requires_imu, "True if the module requires IMU data")
    .def(
      "insert_imu",
      &glim::OdometryEstimationBase::insert_imu,
      py::arg("stamp"),
      py::arg("linear_acc"),
      py::arg("angular_vel"),
      py::call_guard<py::gil_scoped_release>(),
      "Insert a single IMU measurement")
    .def(
      "insert_imu",
      [](glim::OdometryEstimationBase& self, const DoubleArray& imu_data) { insert_imu_batch(self, imu_data); },
      py::arg("imu_data"),
      "Insert a batch of IMU measurements [N, 7] (t, ax, ay, az, wx, wy, wz)")
    .def(
      "insert_frame",
      [](glim::OdometryEstimationBase& self, const glim::PreprocessedFrame::Ptr& frame) {
        std::vector<glim::EstimationFrame::ConstPtr> marginalized;
        glim::EstimationFrame::ConstPtr result;
        {
          py::gil_scoped_release release;
          result = self.insert_frame(frame, marginalized);
        }
        return py::make_tuple(unconst(result), unconst(marginalized));
      },
      py::arg("frame"),
      "Insert a preprocessed point cloud. Returns (estimation result for the latest frame, marginalized frames)")
    .def(
      "get_remaining_frames",
      [](glim::OdometryEstimationBase& self) {
        py::gil_scoped_release release;
        return unconst(self.get_remaining_frames());
      },
      "Pop out the remaining non-marginalized frames (called at the end of the sequence)");

  // glim::OdometryEstimationCPU
  py::class_<glim::OdometryEstimationCPU, glim::OdometryEstimationBase, std::shared_ptr<glim::OdometryEstimationCPU>>(m, "OdometryEstimationCPU", "CPU-based semi-tightly coupled LiDAR-IMU odometry")
    .def(py::init([]() { return std::make_shared<glim::OdometryEstimationCPU>(); }), "Create with parameters loaded from the global config")
    .def(py::init([](const glim::OdometryEstimationCPUParams& params) { return std::make_shared<glim::OdometryEstimationCPU>(params); }), py::arg("params"));

  // glim::OdometryEstimationCT
  py::class_<glim::OdometryEstimationCT, glim::OdometryEstimationBase, std::shared_ptr<glim::OdometryEstimationCT>>(m, "OdometryEstimationCT", "LiDAR-only odometry estimation based on CT-GICP")
    .def(py::init([]() { return std::make_shared<glim::OdometryEstimationCT>(); }), "Create with parameters loaded from the global config")
    .def(py::init([](const glim::OdometryEstimationCTParams& params) { return std::make_shared<glim::OdometryEstimationCT>(params); }), py::arg("params"));

#ifdef PYGLIM_USE_CUDA
  // glim::OdometryEstimationGPU
  py::class_<glim::OdometryEstimationGPU, glim::OdometryEstimationBase, std::shared_ptr<glim::OdometryEstimationGPU>>(m, "OdometryEstimationGPU", "GPU-based tightly coupled LiDAR-IMU odometry")
    .def(py::init([]() { return std::make_shared<glim::OdometryEstimationGPU>(); }), "Create with parameters loaded from the global config")
    .def(py::init([](const glim::OdometryEstimationGPUParams& params) { return std::make_shared<glim::OdometryEstimationGPU>(params); }), py::arg("params"));
#endif

  // glim::AsyncOdometryEstimation
  py::class_<glim::AsyncOdometryEstimation, std::shared_ptr<glim::AsyncOdometryEstimation>>(m, "AsyncOdometryEstimation", "Wrapper to run odometry estimation in a background thread")
    .def(
      py::init([](const std::shared_ptr<glim::OdometryEstimationBase>& odometry, const py::object& enable_imu) {
        const bool imu = enable_imu.is_none() ? odometry->requires_imu() : enable_imu.cast<bool>();
        return std::make_shared<glim::AsyncOdometryEstimation>(odometry, imu);
      }),
      py::arg("odometry"),
      py::arg("enable_imu") = py::none())
    .def(
      "insert_imu",
      &glim::AsyncOdometryEstimation::insert_imu,
      py::arg("stamp"),
      py::arg("linear_acc"),
      py::arg("angular_vel"),
      py::call_guard<py::gil_scoped_release>(),
      "Insert a single IMU measurement")
    .def(
      "insert_imu",
      [](glim::AsyncOdometryEstimation& self, const DoubleArray& imu_data) { insert_imu_batch(self, imu_data); },
      py::arg("imu_data"),
      "Insert a batch of IMU measurements [N, 7] (t, ax, ay, az, wx, wy, wz)")
    .def("insert_frame", &glim::AsyncOdometryEstimation::insert_frame, py::arg("frame"), py::call_guard<py::gil_scoped_release>(), "Insert a preprocessed point cloud")
    .def("workload", &glim::AsyncOdometryEstimation::workload, "Size of the input queue")
    .def("join", &glim::AsyncOdometryEstimation::join, py::call_guard<py::gil_scoped_release>(), "Wait for the odometry estimation thread to process all the input data")
    .def(
      "get_results",
      [](glim::AsyncOdometryEstimation& self) {
        std::vector<glim::EstimationFrame::ConstPtr> estimation_results;
        std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
        {
          py::gil_scoped_release release;
          self.get_results(estimation_results, marginalized_frames);
          // null results are emitted while the initial state estimation has not yet converged
          const auto is_null = [](const auto& frame) { return frame == nullptr; };
          estimation_results.erase(std::remove_if(estimation_results.begin(), estimation_results.end(), is_null), estimation_results.end());
          marginalized_frames.erase(std::remove_if(marginalized_frames.begin(), marginalized_frames.end(), is_null), marginalized_frames.end());
        }
        return py::make_tuple(unconst(estimation_results), unconst(marginalized_frames));
      },
      "Get the estimation results. Returns (estimation results, marginalized frames)");
}
