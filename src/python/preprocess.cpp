#include "pyglim.hpp"

#include <glim/preprocess/cloud_preprocessor.hpp>

void define_preprocess(py::module_& m) {
  // glim::CloudPreprocessorParams
  auto params = py::class_<glim::CloudPreprocessorParams>(m, "CloudPreprocessorParams", "Point cloud preprocessing parameters")
    .def(py::init<>())
    .def_readwrite("distance_near_thresh", &glim::CloudPreprocessorParams::distance_near_thresh, "Minimum distance threshold")
    .def_readwrite("distance_far_thresh", &glim::CloudPreprocessorParams::distance_far_thresh, "Maximum distance threshold")
    .def_readwrite("global_shutter", &glim::CloudPreprocessorParams::global_shutter, "Assume all points in a scan are taken at the same moment")
    .def_readwrite("use_random_grid_downsampling", &glim::CloudPreprocessorParams::use_random_grid_downsampling, "Use random grid downsampling instead of voxel grid")
    .def_readwrite("downsample_resolution", &glim::CloudPreprocessorParams::downsample_resolution, "Downsampling resolution")
    .def_readwrite("downsample_target", &glim::CloudPreprocessorParams::downsample_target, "Target number of points for downsampling")
    .def_readwrite("downsample_rate", &glim::CloudPreprocessorParams::downsample_rate, "Downsampling rate (used for random grid downsampling)")
    .def_readwrite("enable_outlier_removal", &glim::CloudPreprocessorParams::enable_outlier_removal, "Apply statistical outlier removal")
    .def_readwrite("outlier_removal_k", &glim::CloudPreprocessorParams::outlier_removal_k, "Number of neighbors used for outlier removal")
    .def_readwrite("outlier_std_mul_factor", &glim::CloudPreprocessorParams::outlier_std_mul_factor, "Statistical outlier removal std dev threshold multiplier")
    .def_readwrite("enable_cropbox_filter", &glim::CloudPreprocessorParams::enable_cropbox_filter, "Filter out points within box")
    .def_readwrite("crop_bbox_frame", &glim::CloudPreprocessorParams::crop_bbox_frame, "Bounding box reference frame (lidar or imu)")
    .def_readwrite("crop_bbox_min", &glim::CloudPreprocessorParams::crop_bbox_min, "Bounding box min point")
    .def_readwrite("crop_bbox_max", &glim::CloudPreprocessorParams::crop_bbox_max, "Bounding box max point")
    .def_readwrite("k_correspondences", &glim::CloudPreprocessorParams::k_correspondences, "Number of neighboring points")
    .def_readwrite("num_threads", &glim::CloudPreprocessorParams::num_threads, "Number of threads");
  def_isometry_property(params, "T_imu_lidar", &glim::CloudPreprocessorParams::T_imu_lidar, "LiDAR-IMU transformation (4x4 matrix)");

  // glim::PreprocessedFrame
  py::class_<glim::PreprocessedFrame, std::shared_ptr<glim::PreprocessedFrame>>(m, "PreprocessedFrame", "Preprocessed point cloud")
    .def(py::init<>())
    .def("__repr__", [](const glim::PreprocessedFrame& f) { return "<pyglim.PreprocessedFrame stamp=" + std::to_string(f.stamp) + " size=" + std::to_string(f.size()) + ">"; })
    .def("__len__", &glim::PreprocessedFrame::size)
    .def("size", &glim::PreprocessedFrame::size)
    .def_readwrite("stamp", &glim::PreprocessedFrame::stamp, "Timestamp at the beginning of the scan")
    .def_readwrite("scan_end_time", &glim::PreprocessedFrame::scan_end_time, "Timestamp at the end of the scan")
    .def_property_readonly(
      "points",
      [](const glim::PreprocessedFrame& f) { return convert_points(f.points); },
      "Points [N, 3]")
    .def_property_readonly(
      "times",
      [](const glim::PreprocessedFrame& f) { return py::array_t<double>(f.times.size(), f.times.data()); },
      "Per-point timestamps w.r.t. the first point [N]")
    .def_property_readonly(
      "intensities",
      [](const glim::PreprocessedFrame& f) { return py::array_t<double>(f.intensities.size(), f.intensities.data()); },
      "Point intensities [N]")
    .def_readonly("k_neighbors", &glim::PreprocessedFrame::k_neighbors, "Number of neighbors of each point")
    .def_property_readonly(
      "neighbors",
      [](const glim::PreprocessedFrame& f) { return py::array_t<int>(f.neighbors.size(), f.neighbors.data()); },
      "k-nearest neighbor indices of each point [N * k_neighbors]");

  // glim::CloudPreprocessor
  py::class_<glim::CloudPreprocessor, std::shared_ptr<glim::CloudPreprocessor>>(m, "CloudPreprocessor", "Point cloud preprocessor")
    .def(py::init([]() { return std::make_shared<glim::CloudPreprocessor>(); }), "Create with parameters loaded from the global config")
    .def(py::init([](const glim::CloudPreprocessorParams& params) { return std::make_shared<glim::CloudPreprocessor>(params); }), py::arg("params"))
    .def(
      "preprocess",
      [](glim::CloudPreprocessor& self, const glim::RawPoints::ConstPtr& raw_points) {
        py::gil_scoped_release release;
        return self.preprocess(raw_points);
      },
      py::arg("raw_points"),
      "Preprocess a raw point cloud")
    .def(
      "preprocess",
      [](glim::CloudPreprocessor& self, double stamp, const DoubleArray& points, const py::object& times, const py::object& intensities) {
        auto raw_points = std::make_shared<glim::RawPoints>();
        raw_points->stamp = stamp;
        raw_points->points = convert_points(points);
        if (!times.is_none()) {
          raw_points->times = convert_scalars(times.cast<DoubleArray>(), "times", raw_points->points.size());
        } else {
          raw_points->times.assign(raw_points->points.size(), 0.0);
        }
        if (!intensities.is_none()) {
          raw_points->intensities = convert_scalars(intensities.cast<DoubleArray>(), "intensities", raw_points->points.size());
        }

        py::gil_scoped_release release;
        return self.preprocess(raw_points);
      },
      py::arg("stamp"),
      py::arg("points"),
      py::arg("times") = py::none(),
      py::arg("intensities") = py::none(),
      "Preprocess a raw point cloud given as a numpy array [N, 3] or [N, 4]");
}
