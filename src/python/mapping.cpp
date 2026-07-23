#include "pyglim.hpp"

#include <gtsam_points/types/point_cloud.hpp>
#include <glim/mapping/sub_map.hpp>
#include <glim/mapping/sub_mapping.hpp>
#include <glim/mapping/sub_mapping_passthrough.hpp>
#include <glim/mapping/global_mapping.hpp>
#include <glim/mapping/global_mapping_pose_graph.hpp>
#include <glim/mapping/async_sub_mapping.hpp>
#include <glim/mapping/async_global_mapping.hpp>

void define_mapping(py::module_& m) {
  // glim::SubMap
  auto submap = py::class_<glim::SubMap, std::shared_ptr<glim::SubMap>>(m, "SubMap", "SubMap")
    .def("__repr__", [](const glim::SubMap& s) { return "<pyglim.SubMap id=" + std::to_string(s.id) + ">"; })
    .def_readonly("id", &glim::SubMap::id, "SubMap ID")
    .def_readonly("session_id", &glim::SubMap::session_id, "Session ID")
    .def_property_readonly(
      "num_points",
      [](const glim::SubMap& s) -> size_t { return s.frame ? s.frame->size() : 0; },
      "Number of points in the merged submap frame")
    .def_property_readonly(
      "points",
      [](const glim::SubMap& s) {
        if (!s.frame || !s.frame->points) {
          return convert_points(nullptr, 0);
        }
        return convert_points(s.frame->points, s.frame->size());
      },
      "Merged submap points [N, 3] (in the submap origin frame)")
    .def_property_readonly("frames", [](const glim::SubMap& s) { return unconst(s.frames); }, "Optimized odometry frames")
    .def_property_readonly("odom_frames", [](const glim::SubMap& s) { return unconst(s.odom_frames); }, "Original odometry frames")
    .def("origin_frame", [](const glim::SubMap& s) { return unconst(s.origin_frame()); }, "Get the origin frame")
    .def("origin_odom_frame", [](const glim::SubMap& s) { return unconst(s.origin_odom_frame()); }, "Get the origin odometry frame")
    .def("drop_frame_points", &glim::SubMap::drop_frame_points, "Remove point clouds of the odometry frames to save memory")
    .def("save", &glim::SubMap::save, py::arg("path"), py::call_guard<py::gil_scoped_release>(), "Save the submap")
    .def_static("load", &glim::SubMap::load, py::arg("path"), py::call_guard<py::gil_scoped_release>(), "Load a submap from storage");
  def_isometry_property(submap, "T_world_origin", &glim::SubMap::T_world_origin, "Origin frame pose w.r.t. the world (4x4 matrix)");
  def_isometry_property(submap, "T_origin_endpoint_L", &glim::SubMap::T_origin_endpoint_L, "First frame pose w.r.t. the origin (4x4 matrix)");
  def_isometry_property(submap, "T_origin_endpoint_R", &glim::SubMap::T_origin_endpoint_R, "Last frame pose w.r.t. the origin (4x4 matrix)");

  // glim::SubMappingParams
  py::class_<glim::SubMappingParams>(m, "SubMappingParams", "Sub mapping parameters")
    .def(py::init<>(), "Load parameters from the global config")
    .def_readwrite("enable_gpu", &glim::SubMappingParams::enable_gpu)
    .def_readwrite("enable_imu", &glim::SubMappingParams::enable_imu)
    .def_readwrite("enable_optimization", &glim::SubMappingParams::enable_optimization)
    .def_readwrite("max_num_keyframes", &glim::SubMappingParams::max_num_keyframes)
    .def_readwrite("keyframe_update_strategy", &glim::SubMappingParams::keyframe_update_strategy)
    .def_readwrite("keyframe_update_min_points", &glim::SubMappingParams::keyframe_update_min_points)
    .def_readwrite("keyframe_update_interval_rot", &glim::SubMappingParams::keyframe_update_interval_rot)
    .def_readwrite("keyframe_update_interval_trans", &glim::SubMappingParams::keyframe_update_interval_trans)
    .def_readwrite("max_keyframe_overlap", &glim::SubMappingParams::max_keyframe_overlap)
    .def_readwrite("create_between_factors", &glim::SubMappingParams::create_between_factors)
    .def_readwrite("between_registration_type", &glim::SubMappingParams::between_registration_type)
    .def_readwrite("registration_error_factor_type", &glim::SubMappingParams::registration_error_factor_type)
    .def_readwrite("keyframe_randomsampling_rate", &glim::SubMappingParams::keyframe_randomsampling_rate)
    .def_readwrite("keyframe_voxel_resolution", &glim::SubMappingParams::keyframe_voxel_resolution)
    .def_readwrite("keyframe_voxelmap_levels", &glim::SubMappingParams::keyframe_voxelmap_levels)
    .def_readwrite("keyframe_voxelmap_scaling_factor", &glim::SubMappingParams::keyframe_voxelmap_scaling_factor)
    .def_readwrite("submap_downsample_resolution", &glim::SubMappingParams::submap_downsample_resolution)
    .def_readwrite("submap_voxel_resolution", &glim::SubMappingParams::submap_voxel_resolution)
    .def_readwrite("submap_target_num_points", &glim::SubMappingParams::submap_target_num_points);

  // glim::SubMappingPassthroughParams
  py::class_<glim::SubMappingPassthroughParams>(m, "SubMappingPassthroughParams", "SubMappingPassthrough parameters")
    .def(py::init<>(), "Load parameters from the global config")
    .def_readwrite("keyframe_update_interval_rot", &glim::SubMappingPassthroughParams::keyframe_update_interval_rot)
    .def_readwrite("keyframe_update_interval_trans", &glim::SubMappingPassthroughParams::keyframe_update_interval_trans)
    .def_readwrite("max_num_keyframes", &glim::SubMappingPassthroughParams::max_num_keyframes)
    .def_readwrite("max_num_voxels", &glim::SubMappingPassthroughParams::max_num_voxels)
    .def_readwrite("adaptive_max_num_voxels", &glim::SubMappingPassthroughParams::adaptive_max_num_voxels)
    .def_readwrite("submap_target_num_points", &glim::SubMappingPassthroughParams::submap_target_num_points)
    .def_readwrite("submap_voxel_resolution", &glim::SubMappingPassthroughParams::submap_voxel_resolution)
    .def_readwrite("min_dist_in_voxel", &glim::SubMappingPassthroughParams::min_dist_in_voxel)
    .def_readwrite("max_num_points_in_voxel", &glim::SubMappingPassthroughParams::max_num_points_in_voxel);

  // glim::SubMappingBase
  py::class_<glim::SubMappingBase, std::shared_ptr<glim::SubMappingBase>>(m, "SubMappingBase", "Sub mapping base class")
    .def_static("load_module", &glim::SubMappingBase::load_module, py::arg("so_name"), "Load a sub mapping module from a shared library (e.g., libsub_mapping.so)")
    .def(
      "insert_imu",
      &glim::SubMappingBase::insert_imu,
      py::arg("stamp"),
      py::arg("linear_acc"),
      py::arg("angular_vel"),
      py::call_guard<py::gil_scoped_release>(),
      "Insert a single IMU measurement")
    .def(
      "insert_imu",
      [](glim::SubMappingBase& self, const DoubleArray& imu_data) { insert_imu_batch(self, imu_data); },
      py::arg("imu_data"),
      "Insert a batch of IMU measurements [N, 7] (t, ax, ay, az, wx, wy, wz)")
    .def("insert_frame", &glim::SubMappingBase::insert_frame, py::arg("frame"), py::call_guard<py::gil_scoped_release>(), "Insert a marginalized odometry estimation frame")
    .def("get_submaps", &glim::SubMappingBase::get_submaps, py::call_guard<py::gil_scoped_release>(), "Get the created submaps")
    .def(
      "submit_end_of_sequence",
      &glim::SubMappingBase::submit_end_of_sequence,
      py::call_guard<py::gil_scoped_release>(),
      "Tell the end of sequence and collect the remaining submap data");

  // glim::SubMapping
  py::class_<glim::SubMapping, glim::SubMappingBase, std::shared_ptr<glim::SubMapping>>(m, "SubMapping", "Sub mapping")
    .def(py::init([]() { return std::make_shared<glim::SubMapping>(); }), "Create with parameters loaded from the global config")
    .def(py::init([](const glim::SubMappingParams& params) { return std::make_shared<glim::SubMapping>(params); }), py::arg("params"));

  // glim::SubMappingPassthrough
  py::class_<glim::SubMappingPassthrough, glim::SubMappingBase, std::shared_ptr<glim::SubMappingPassthrough>>(m, "SubMappingPassthrough", "Sub mapping based on simple voxel-based frame merging")
    .def(py::init([]() { return std::make_shared<glim::SubMappingPassthrough>(); }), "Create with parameters loaded from the global config")
    .def(py::init([](const glim::SubMappingPassthroughParams& params) { return std::make_shared<glim::SubMappingPassthrough>(params); }), py::arg("params"));

  // glim::GlobalMappingParams
  py::class_<glim::GlobalMappingParams>(m, "GlobalMappingParams", "Global mapping parameters")
    .def(py::init<>(), "Load parameters from the global config")
    .def_readwrite("enable_gpu", &glim::GlobalMappingParams::enable_gpu)
    .def_readwrite("enable_imu", &glim::GlobalMappingParams::enable_imu)
    .def_readwrite("enable_optimization", &glim::GlobalMappingParams::enable_optimization)
    .def_readwrite("enable_between_factors", &glim::GlobalMappingParams::enable_between_factors)
    .def_readwrite("between_registration_type", &glim::GlobalMappingParams::between_registration_type)
    .def_readwrite("registration_error_factor_type", &glim::GlobalMappingParams::registration_error_factor_type)
    .def_readwrite("submap_voxel_resolution", &glim::GlobalMappingParams::submap_voxel_resolution)
    .def_readwrite("submap_voxel_resolution_max", &glim::GlobalMappingParams::submap_voxel_resolution_max)
    .def_readwrite("submap_voxel_resolution_dmin", &glim::GlobalMappingParams::submap_voxel_resolution_dmin)
    .def_readwrite("submap_voxel_resolution_dmax", &glim::GlobalMappingParams::submap_voxel_resolution_dmax)
    .def_readwrite("submap_voxelmap_levels", &glim::GlobalMappingParams::submap_voxelmap_levels)
    .def_readwrite("submap_voxelmap_scaling_factor", &glim::GlobalMappingParams::submap_voxelmap_scaling_factor)
    .def_readwrite("randomsampling_rate", &glim::GlobalMappingParams::randomsampling_rate)
    .def_readwrite("max_implicit_loop_distance", &glim::GlobalMappingParams::max_implicit_loop_distance)
    .def_readwrite("min_implicit_loop_overlap", &glim::GlobalMappingParams::min_implicit_loop_overlap)
    .def_readwrite("use_isam2_dogleg", &glim::GlobalMappingParams::use_isam2_dogleg)
    .def_readwrite("isam2_relinearize_skip", &glim::GlobalMappingParams::isam2_relinearize_skip)
    .def_readwrite("isam2_relinearize_thresh", &glim::GlobalMappingParams::isam2_relinearize_thresh)
    .def_readwrite("init_pose_damping_scale", &glim::GlobalMappingParams::init_pose_damping_scale);

  // glim::GlobalMappingPoseGraphParams
  py::class_<glim::GlobalMappingPoseGraphParams>(m, "GlobalMappingPoseGraphParams", "Pose-graph-based global mapping parameters")
    .def(py::init<>(), "Load parameters from the global config")
    .def_readwrite("enable_optimization", &glim::GlobalMappingPoseGraphParams::enable_optimization)
    .def_readwrite("registration_type", &glim::GlobalMappingPoseGraphParams::registration_type)
    .def_readwrite("min_travel_dist", &glim::GlobalMappingPoseGraphParams::min_travel_dist)
    .def_readwrite("max_neighbor_dist", &glim::GlobalMappingPoseGraphParams::max_neighbor_dist)
    .def_readwrite("min_inliear_fraction", &glim::GlobalMappingPoseGraphParams::min_inliear_fraction)
    .def_readwrite("subsample_target", &glim::GlobalMappingPoseGraphParams::subsample_target)
    .def_readwrite("subsample_rate", &glim::GlobalMappingPoseGraphParams::subsample_rate)
    .def_readwrite("gicp_max_correspondence_dist", &glim::GlobalMappingPoseGraphParams::gicp_max_correspondence_dist)
    .def_readwrite("vgicp_voxel_resolution", &glim::GlobalMappingPoseGraphParams::vgicp_voxel_resolution)
    .def_readwrite("odom_factor_stddev", &glim::GlobalMappingPoseGraphParams::odom_factor_stddev)
    .def_readwrite("loop_factor_stddev", &glim::GlobalMappingPoseGraphParams::loop_factor_stddev)
    .def_readwrite("loop_factor_robust_width", &glim::GlobalMappingPoseGraphParams::loop_factor_robust_width)
    .def_readwrite("loop_candidate_buffer_size", &glim::GlobalMappingPoseGraphParams::loop_candidate_buffer_size)
    .def_readwrite("loop_candidate_eval_per_thread", &glim::GlobalMappingPoseGraphParams::loop_candidate_eval_per_thread)
    .def_readwrite("use_isam2_dogleg", &glim::GlobalMappingPoseGraphParams::use_isam2_dogleg)
    .def_readwrite("isam2_relinearize_skip", &glim::GlobalMappingPoseGraphParams::isam2_relinearize_skip)
    .def_readwrite("isam2_relinearize_thresh", &glim::GlobalMappingPoseGraphParams::isam2_relinearize_thresh)
    .def_readwrite("init_pose_damping_scale", &glim::GlobalMappingPoseGraphParams::init_pose_damping_scale)
    .def_readwrite("num_threads", &glim::GlobalMappingPoseGraphParams::num_threads);

  // glim::GlobalMappingBase
  py::class_<glim::GlobalMappingBase, std::shared_ptr<glim::GlobalMappingBase>>(m, "GlobalMappingBase", "Global mapping base class")
    .def_static("load_module", &glim::GlobalMappingBase::load_module, py::arg("so_name"), "Load a global mapping module from a shared library (e.g., libglobal_mapping.so)")
    .def(
      "insert_imu",
      &glim::GlobalMappingBase::insert_imu,
      py::arg("stamp"),
      py::arg("linear_acc"),
      py::arg("angular_vel"),
      py::call_guard<py::gil_scoped_release>(),
      "Insert a single IMU measurement")
    .def(
      "insert_imu",
      [](glim::GlobalMappingBase& self, const DoubleArray& imu_data) { insert_imu_batch(self, imu_data); },
      py::arg("imu_data"),
      "Insert a batch of IMU measurements [N, 7] (t, ax, ay, az, wx, wy, wz)")
    .def("insert_submap", &glim::GlobalMappingBase::insert_submap, py::arg("submap"), py::call_guard<py::gil_scoped_release>(), "Insert a submap")
    .def("find_overlapping_submaps", &glim::GlobalMappingBase::find_overlapping_submaps, py::arg("min_overlap"), py::call_guard<py::gil_scoped_release>(), "Find new overlapping submaps")
    .def("optimize", &glim::GlobalMappingBase::optimize, py::call_guard<py::gil_scoped_release>(), "Perform optimization")
    .def("recover_graph", &glim::GlobalMappingBase::recover_graph, py::call_guard<py::gil_scoped_release>(), "Detect and recover graph corruption")
    .def("save", &glim::GlobalMappingBase::save, py::arg("path"), py::call_guard<py::gil_scoped_release>(), "Save the mapping result")
    .def(
      "export_points",
      [](glim::GlobalMappingBase& self) {
        gtsam_points::PointCloud::Ptr points;
        {
          py::gil_scoped_release release;
          points = self.export_points();
        }
        if (!points || !points->points) {
          return convert_points(nullptr, 0);
        }
        return convert_points(points->points, points->size());
      },
      "Export all the submap points as a numpy array [N, 3]");

  // glim::GlobalMapping
  py::class_<glim::GlobalMapping, glim::GlobalMappingBase, std::shared_ptr<glim::GlobalMapping>>(m, "GlobalMapping", "Global mapping")
    .def(py::init([]() { return std::make_shared<glim::GlobalMapping>(); }), "Create with parameters loaded from the global config")
    .def(py::init([](const glim::GlobalMappingParams& params) { return std::make_shared<glim::GlobalMapping>(params); }), py::arg("params"))
    .def("load", &glim::GlobalMapping::load, py::arg("path"), py::call_guard<py::gil_scoped_release>(), "Load a mapping result from a dumped directory");

  // glim::GlobalMappingPoseGraph
  py::class_<glim::GlobalMappingPoseGraph, glim::GlobalMappingBase, std::shared_ptr<glim::GlobalMappingPoseGraph>>(m, "GlobalMappingPoseGraph", "Pose-graph-based global mapping")
    .def(py::init([]() { return std::make_shared<glim::GlobalMappingPoseGraph>(); }), "Create with parameters loaded from the global config")
    .def(py::init([](const glim::GlobalMappingPoseGraphParams& params) { return std::make_shared<glim::GlobalMappingPoseGraph>(params); }), py::arg("params"));

  // glim::AsyncSubMapping
  py::class_<glim::AsyncSubMapping, std::shared_ptr<glim::AsyncSubMapping>>(m, "AsyncSubMapping", "Wrapper to run sub mapping in a background thread")
    .def(py::init([](const std::shared_ptr<glim::SubMappingBase>& sub_mapping) { return std::make_shared<glim::AsyncSubMapping>(sub_mapping); }), py::arg("sub_mapping"))
    .def(
      "insert_imu",
      &glim::AsyncSubMapping::insert_imu,
      py::arg("stamp"),
      py::arg("linear_acc"),
      py::arg("angular_vel"),
      py::call_guard<py::gil_scoped_release>(),
      "Insert a single IMU measurement")
    .def(
      "insert_imu",
      [](glim::AsyncSubMapping& self, const DoubleArray& imu_data) { insert_imu_batch(self, imu_data); },
      py::arg("imu_data"),
      "Insert a batch of IMU measurements [N, 7] (t, ax, ay, az, wx, wy, wz)")
    .def("insert_frame", &glim::AsyncSubMapping::insert_frame, py::arg("frame"), py::call_guard<py::gil_scoped_release>(), "Insert a marginalized odometry estimation frame")
    .def("workload", &glim::AsyncSubMapping::workload, "Size of the input queue")
    .def("join", &glim::AsyncSubMapping::join, py::call_guard<py::gil_scoped_release>(), "Wait for the sub mapping thread to process all the input data")
    .def("get_results", &glim::AsyncSubMapping::get_results, py::call_guard<py::gil_scoped_release>(), "Get the created submaps");

  // glim::AsyncGlobalMapping
  py::class_<glim::AsyncGlobalMapping, std::shared_ptr<glim::AsyncGlobalMapping>>(m, "AsyncGlobalMapping", "Wrapper to run global mapping in a background thread")
    .def(
      py::init([](const std::shared_ptr<glim::GlobalMappingBase>& global_mapping, int optimization_interval_sec) {
        return std::make_shared<glim::AsyncGlobalMapping>(global_mapping, optimization_interval_sec);
      }),
      py::arg("global_mapping"),
      py::arg("optimization_interval_sec") = 5)
    .def(
      "insert_imu",
      &glim::AsyncGlobalMapping::insert_imu,
      py::arg("stamp"),
      py::arg("linear_acc"),
      py::arg("angular_vel"),
      py::call_guard<py::gil_scoped_release>(),
      "Insert a single IMU measurement")
    .def(
      "insert_imu",
      [](glim::AsyncGlobalMapping& self, const DoubleArray& imu_data) { insert_imu_batch(self, imu_data); },
      py::arg("imu_data"),
      "Insert a batch of IMU measurements [N, 7] (t, ax, ay, az, wx, wy, wz)")
    .def("insert_submap", &glim::AsyncGlobalMapping::insert_submap, py::arg("submap"), py::call_guard<py::gil_scoped_release>(), "Insert a submap")
    .def("workload", &glim::AsyncGlobalMapping::workload, "Size of the input queue")
    .def("join", &glim::AsyncGlobalMapping::join, py::call_guard<py::gil_scoped_release>(), "Wait for the global mapping thread to process all the input data")
    .def("save", &glim::AsyncGlobalMapping::save, py::arg("path"), py::call_guard<py::gil_scoped_release>(), "Save the mapping result (call after join())")
    .def(
      "export_points",
      [](glim::AsyncGlobalMapping& self) {
        gtsam_points::PointCloud::Ptr points;
        {
          py::gil_scoped_release release;
          points = self.export_points();
        }
        if (!points || !points->points) {
          return convert_points(nullptr, 0);
        }
        return convert_points(points->points, points->size());
      },
      "Export all the submap points as a numpy array [N, 3]");
}
