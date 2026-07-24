#include "pyglim.hpp"

#include <glim/batch/batched_imu_measurements.hpp>
#include <glim/batch/batched_raw_points.hpp>
#include <glim/batch/batched_preprocessed_frame.hpp>
#include <glim/batch/batched_estimation_frame.hpp>
#include <glim/batch/batched_time_keeper.hpp>
#include <glim/batch/batched_cloud_preprocessor.hpp>
#include <glim/batch/batched_odometry_estimation.hpp>

void define_batch(py::module_& m) {
  // glim::BatchedIMUMeasurements
  py::class_<glim::BatchedIMUMeasurements, std::shared_ptr<glim::BatchedIMUMeasurements>> imu_measurements(m, "BatchedIMUMeasurements", "Batched IMU measurements");
  imu_measurements.def(py::init<int>(), py::arg("batch_size"), "Create a batched IMU measurements container with the specified batch size");
  imu_measurements.def("size", &glim::BatchedIMUMeasurements::size, "Get the batch size");
  imu_measurements.def("__len__", &glim::BatchedIMUMeasurements::size, "Get the batch size");
  imu_measurements.def(
    "insert",
    py::overload_cast<int, double, const Eigen::Vector3d&, const Eigen::Vector3d&>(&glim::BatchedIMUMeasurements::insert),
    py::arg("batch_index"),
    py::arg("stamp"),
    py::arg("linear_acc"),
    py::arg("angular_vel"),
    "Insert a single IMU measurement into the specified batch index");
  imu_measurements.def(
    "insert",
    py::overload_cast<int, const Eigen::Matrix<double, 7, 1>&>(&glim::BatchedIMUMeasurements::insert),
    py::arg("batch_index"),
    py::arg("imu_measurement"),
    "Insert a single IMU measurement ([t, ax, ay, az, wx, wy, wz]) into the specified batch index");
  imu_measurements.def(
    "insert",
    py::overload_cast<int, const Eigen::Matrix<double, -1, 7>&>(&glim::BatchedIMUMeasurements::insert),
    py::arg("batch_index"),
    py::arg("imu_measurements"),
    "Insert a batch of IMU measurements ([N, 7] with columns [t, ax, ay, az, wx, wy, wz]) into the specified batch index");
  // batch input for ndarray.float [B, N, 7] (B=batch size, N=number of measurements in each batch)
  imu_measurements.def(
    "insert",
    [](glim::BatchedIMUMeasurements& self, const DoubleArray& imu_data) {
      if (imu_data.ndim() != 3) {
        throw std::invalid_argument("imu_data must be [B, N, 7] (ndim=" + std::to_string(imu_data.ndim()) + ")");
      }
      if (imu_data.shape(2) != 7) {
        throw std::invalid_argument("imu_data must have 7 columns [t, ax, ay, az, wx, wy, wz] (cols=" + std::to_string(imu_data.shape(2)) + ")");
      }
      validate_batch_size("imu_data", imu_data.shape(0), self.size());

      const auto view = imu_data.unchecked<3>();
      for (int b = 0; b < self.size(); b++) {
        for (py::ssize_t i = 0; i < view.shape(1); i++) {
          if (std::isnan(view(b, i, 0))) {
            break;
          }
          Eigen::Matrix<double, 7, 1> imu;
          imu << view(b, i, 0), view(b, i, 1), view(b, i, 2), view(b, i, 3), view(b, i, 4), view(b, i, 5), view(b, i, 6);
          self[b].emplace_back(imu);
        }
      }
    },
    py::arg("imu_data"),
    "Insert a batch of IMU measurements ([B, N, 7] with [t, ax, ay, az, wx, wy, wz]). If batches have different numbers of measurements, values for missing measurements should "
    "be set to NaN.");

  // glim::BatchedRawPoints
  py::class_<glim::BatchedRawPoints, std::shared_ptr<glim::BatchedRawPoints>> raw_points(m, "BatchedRawPoints", "Batched raw points");
  raw_points.def(py::init<int>(), py::arg("batch_size"), "Create a batched raw points container with the specified batch size");
  raw_points.def("size", &glim::BatchedRawPoints::size, "Get the batch size");
  raw_points.def("__len__", &glim::BatchedRawPoints::size, "Get the batch size");
  raw_points.def(
    "__getitem__",
    [](glim::BatchedRawPoints& self, int index) {
      if (index < 0 || index >= self.size()) {
        throw std::out_of_range("Index out of range");
      }
      return self[index];
    },
    py::arg("index"),
    "Get the raw points at the specified batch index");

  // Index-wise setters for stamp, times, intensities, points, colors, and rings
  raw_points.def(
    "set_stamp",
    [](glim::BatchedRawPoints& self, int batch_index, const double& stamp) {
      validate_batch_index(batch_index, self.size());
      self[batch_index]->stamp = stamp;
    },
    py::arg("batch_index"),
    py::arg("stamp"),
    "Set the timestamp of the raw points at the specified batch index");
  raw_points.def(
    "set_times",
    [](glim::BatchedRawPoints& self, int batch_index, DoubleArray times) {
      validate_batch_index(batch_index, self.size());
      self[batch_index]->times = convert_scalar_frame(times, "times");
    },
    py::arg("batch_index"),
    py::arg("times"),
    "Set the per-point timestamps for the raw points at the specified batch index. The input should be a [N] or [N, 1] array, where N is the number of points in the batch.");
  raw_points.def(
    "set_intensities",
    [](glim::BatchedRawPoints& self, int batch_index, DoubleArray intensities) {
      validate_batch_index(batch_index, self.size());
      self[batch_index]->intensities = convert_scalar_frame(intensities, "intensities");
    },
    py::arg("batch_index"),
    py::arg("intensities"),
    "Set the per-point intensities for the raw points at the specified batch index. The input should be a [N] or [N, 1] array, where N is the number of points in the batch.");
  raw_points.def(
    "set_points",
    [](glim::BatchedRawPoints& self, int batch_index, DoubleArray points) {
      validate_batch_index(batch_index, self.size());
      self[batch_index]->points = convert_point_frame(points, "points", false);
    },
    py::arg("batch_index"),
    py::arg("points"),
    "Set the point coordinates for the raw points at the specified batch index. The input should be a [N, 3] or [N, 4], where N is the number of points in the batch.");
  raw_points.def(
    "set_colors",
    [](glim::BatchedRawPoints& self, int batch_index, DoubleArray colors) {
      validate_batch_index(batch_index, self.size());
      self[batch_index]->colors = convert_point_frame(colors, "colors", true);
    },
    py::arg("batch_index"),
    py::arg("colors"),
    "Set the point colors for the raw points at the specified batch index. The input should be a [N, 3] or [N, 4], where N is the number of points in the batch.");
  raw_points.def(
    "set_rings",
    [](glim::BatchedRawPoints& self, int batch_index, IntArray rings) {
      validate_batch_index(batch_index, self.size());
      self[batch_index]->rings = convert_ring_frame(rings);
    },
    py::arg("batch_index"),
    py::arg("rings"),
    "Set the per-point ring numbers for the raw points at the specified batch index. The input should be a [N] or [N, 1] array, where N is the number of points in the batch.");

  // Batch setters for stamps, times, intensities, points, colors, and rings
  raw_points.def(
    "set_stamps",
    [](glim::BatchedRawPoints& self, DoubleArray stamps) {
      if (stamps.ndim() != 1 && !(stamps.ndim() == 2 && stamps.shape(1) == 1)) {
        throw std::invalid_argument("stamps must be [B] or [B, 1] (ndim=" + std::to_string(stamps.ndim()) + ")");
      }
      validate_batch_size("stamps", stamps.shape(0), self.size());
      const double* data = stamps.data();
      for (int b = 0; b < self.size(); b++) {
        self[b]->stamp = data[b];
      }
    },
    py::arg("stamps"),
    "Set the batch of timestamps for the raw points. The input should be a [B] or [B, 1] array, where B is the batch size.");
  raw_points.def(
    "set_times",
    [](glim::BatchedRawPoints& self, DoubleArray times) {
      auto batch = convert_scalar_batch(times, "times", self.size());
      for (int b = 0; b < self.size(); b++) {
        self[b]->times = std::move(batch[b]);
      }
    },
    py::arg("times"),
    "Set the batch of per-point timestamps for the raw points. The input should be a [B, N] or [B, N, 1] array, where B is the batch size and N is the number of points in each "
    "batch. If frames have different numbers of points, values for missing points should be set to NaN.");
  raw_points.def(
    "set_intensities",
    [](glim::BatchedRawPoints& self, DoubleArray intensities) {
      auto batch = convert_scalar_batch(intensities, "intensities", self.size());
      for (int b = 0; b < self.size(); b++) {
        self[b]->intensities = std::move(batch[b]);
      }
    },
    py::arg("intensities"),
    "Set the batch of per-point intensities for the raw points. The input should be a [B, N] or [B, N, 1] array, where B is the batch size and N is the number of points in "
    "each batch. If frames have different numbers of points, values for missing points should be set to NaN.");
  raw_points.def(
    "set_points",
    [](glim::BatchedRawPoints& self, DoubleArray points) {
      auto batch = convert_point_batch(points, "points", self.size(), false);
      for (int b = 0; b < self.size(); b++) {
        self[b]->points = std::move(batch[b]);
      }
    },
    py::arg("points"),
    "Set the batch of per-point coordinates for the raw points. The input should be a [B, N, 3] or [B, N, 4], where B is the batch size and N is the number of points in each "
    "batch. If frames have different numbers of points, values for missing points should be set to NaN.");
  raw_points.def(
    "set_colors",
    [](glim::BatchedRawPoints& self, DoubleArray colors) {
      auto batch = convert_point_batch(colors, "colors", self.size(), true);
      for (int b = 0; b < self.size(); b++) {
        self[b]->colors = std::move(batch[b]);
      }
    },
    py::arg("colors"),
    "Set the batch of per-point colors for the raw points. The input should be a [B, N, 3] or [B, N, 4], where B is the batch size and N is the number of points in each "
    "batch. If frames have different numbers of points, values for missing points should be set to NaN.");
  raw_points.def(
    "set_rings",
    [](glim::BatchedRawPoints& self, IntArray rings) {
      auto batch = convert_ring_batch(rings, self.size());
      for (int b = 0; b < self.size(); b++) {
        self[b]->rings = std::move(batch[b]);
      }
    },
    py::arg("rings"),
    "Set the batch of per-point ring numbers for the raw points. The input should be a [B, N] or [B, N, 1] array, where B is the batch size and N is the number of points in "
    "each batch. If frames have different numbers of points, values for missing points should be set to -1.");

  // glim::BatchedPreprocessedFrame
  py::class_<glim::BatchedPreprocessedFrame, std::shared_ptr<glim::BatchedPreprocessedFrame>> preprocessed_frame(m, "BatchedPreprocessedFrame", "Batched preprocessed frame");
  preprocessed_frame.def(py::init<int>(), py::arg("batch_size"), "Create a batched preprocessed frame container with the specified batch size");
  preprocessed_frame.def("size", &glim::BatchedPreprocessedFrame::size, "Get the batch size");
  preprocessed_frame.def("__len__", &glim::BatchedPreprocessedFrame::size, "Get the batch size");
  preprocessed_frame.def(
    "__getitem__",
    [](glim::BatchedPreprocessedFrame& self, int index) {
      if (index < 0 || index >= self.size()) {
        throw std::out_of_range("Index out of range");
      }
      return self[index];
    },
    py::arg("index"),
    "Get the preprocessed frame at the specified batch index");

  // glim::BatchedEstimationFrame
  py::class_<glim::BatchedEstimationFrame, std::shared_ptr<glim::BatchedEstimationFrame>> estimation_frame(m, "BatchedEstimationFrame", "Batched estimation frame");
  estimation_frame.def(py::init<int>(), py::arg("batch_size"), "Create a batched estimation frame container with the specified batch size");
  estimation_frame.def("size", &glim::BatchedEstimationFrame::size, "Get the batch size");
  estimation_frame.def("__len__", &glim::BatchedEstimationFrame::size, "Get the batch size");
  estimation_frame.def(
    "__getitem__",
    [](glim::BatchedEstimationFrame& self, int index) {
      if (index < 0 || index >= self.size()) {
        throw std::out_of_range("Index out of range");
      }
      return self[index];
    },
    py::arg("index"),
    "Get the estimation frame at the specified batch index");

  // glim::BatchedTimeKeeper
  py::class_<glim::BatchedTimeKeeper, std::shared_ptr<glim::BatchedTimeKeeper>> time_keeper(m, "BatchedTimeKeeper", "Batched time keeper");
  time_keeper.def(py::init<int, int>(), py::arg("batch_size"), py::arg("num_threads") = -1, "Create a batched time keeper with the specified batch size and number of threads");
  time_keeper.def("validate_imu_stamp", &glim::BatchedTimeKeeper::validate_imu_stamp, py::arg("batched_imu"), "Validate the timestamps of the batched IMU measurements");
  time_keeper.def("process", &glim::BatchedTimeKeeper::process, py::arg("batched_points"), "Process the batched raw points to compute timestamps and durations for each batch");

  // glim::BatchedCloudPreprocessor
  py::class_<glim::BatchedCloudPreprocessor, std::shared_ptr<glim::BatchedCloudPreprocessor>> cloud_preprocessor(m, "BatchedCloudPreprocessor", "Batched cloud preprocessor");
  cloud_preprocessor
    .def(py::init<int, int>(), py::arg("batch_size"), py::arg("num_threads") = -1, "Create a batched cloud preprocessor with the specified batch size and number of threads");
  cloud_preprocessor.def("process", &glim::BatchedCloudPreprocessor::process, py::arg("batched_points"), "Process the batched raw points to produce a batched preprocessed frame");

  // glim::BatchedOdometryEstimation
  py::class_<glim::BatchedOdometryEstimation, std::shared_ptr<glim::BatchedOdometryEstimation>> odometry_estimation(m, "BatchedOdometryEstimation", "Batched odometry estimation");
  odometry_estimation.def(
    py::init<int, const std::string&, int>(),
    py::arg("batch_size"),
    py::arg("so_name"),
    py::arg("num_threads") = -1,
    "Create a batched odometry estimation with the specified batch size, shared object name, and number of threads");
  odometry_estimation.def("insert_imu", &glim::BatchedOdometryEstimation::insert_imu, py::arg("batched_imu"), "Insert batched IMU measurements into the odometry estimation");
  odometry_estimation.def(
    "insert_frame",
    [](glim::BatchedOdometryEstimation& self, const glim::BatchedPreprocessedFrame::Ptr& batched_frame) {
      std::vector<std::vector<glim::EstimationFrame::ConstPtr>> marginalized_states;
      auto latest = self.insert_frame(batched_frame, marginalized_states);
      return std::make_pair(latest, std::move(marginalized_states));
    },
    py::arg("batched_frame"),
    "Insert a batched preprocessed frame into the odometry estimation and return the corresponding batched estimation frame. The marginalized states are not returned in this "
    "overload.");
}
