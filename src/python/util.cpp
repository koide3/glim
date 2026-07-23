#include "pyglim.hpp"

#include <spdlog/spdlog.h>

#include <glim/util/config.hpp>
#include <glim/util/raw_points.hpp>
#include <glim/util/time_keeper.hpp>
#include <glim/util/extension_module.hpp>

/// @brief Create RawPoints from numpy arrays
static glim::RawPoints::Ptr create_raw_points(double stamp, const DoubleArray& points, const py::object& times, const py::object& intensities) {
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

  return raw_points;
}

void define_util(py::module_& m) {
  // set_verbosity
  m.def(
    "set_verbosity",
    [](const std::string& level) { spdlog::set_level(spdlog::level::from_str(level)); },
    "Set the logging level (trace/debug/info/warn/error/critical/off)",
    py::arg("level"));

  // glim::Config
  py::class_<glim::Config>(m, "Config", "Configuration loader from JSON files")
    .def(py::init<std::string>(), py::arg("config_filename"))
    .def("has_param", &glim::Config::has_param, py::arg("module_name"), py::arg("param_name"))
    // note: the bool overload must precede the int overload because a Python bool is also an int
    .def("param", [](const glim::Config& c, const std::string& m, const std::string& p, const bool& d) { return c.param(m, p, d); }, py::arg("module_name"), py::arg("param_name"), py::arg("default_value"))
    .def("param", [](const glim::Config& c, const std::string& m, const std::string& p, const int& d) { return c.param(m, p, d); }, py::arg("module_name"), py::arg("param_name"), py::arg("default_value"))
    .def("param", [](const glim::Config& c, const std::string& m, const std::string& p, const double& d) { return c.param(m, p, d); }, py::arg("module_name"), py::arg("param_name"), py::arg("default_value"))
    .def("param", [](const glim::Config& c, const std::string& m, const std::string& p, const std::string& d) { return c.param(m, p, d); }, py::arg("module_name"), py::arg("param_name"), py::arg("default_value"))
    .def("param", [](const glim::Config& c, const std::string& m, const std::string& p, const std::vector<double>& d) { return c.param(m, p, d); }, py::arg("module_name"), py::arg("param_name"), py::arg("default_value"))
    .def("param", [](const glim::Config& c, const std::string& m, const std::string& p, const std::vector<std::string>& d) { return c.param(m, p, d); }, py::arg("module_name"), py::arg("param_name"), py::arg("default_value"))
    .def("override_param", &glim::Config::override_param<bool>, py::arg("module_name"), py::arg("param_name"), py::arg("value"))
    .def("override_param", &glim::Config::override_param<int>, py::arg("module_name"), py::arg("param_name"), py::arg("value"))
    .def("override_param", &glim::Config::override_param<double>, py::arg("module_name"), py::arg("param_name"), py::arg("value"))
    .def("override_param", &glim::Config::override_param<std::string>, py::arg("module_name"), py::arg("param_name"), py::arg("value"))
    .def("save", &glim::Config::save, py::arg("path"));

  // glim::GlobalConfig
  py::class_<glim::GlobalConfig, glim::Config, std::unique_ptr<glim::GlobalConfig, py::nodelete>>(m, "GlobalConfig", "Global configuration singleton")
    .def_static(
      "instance",
      [](const std::string& config_path, bool override_path) -> glim::GlobalConfig* { return glim::GlobalConfig::instance(config_path, override_path); },
      py::arg("config_path") = "",
      py::arg("override_path") = false,
      py::return_value_policy::reference)
    .def_static("get_config_path", &glim::GlobalConfig::get_config_path, py::arg("config_name"))
    .def("dump", &glim::GlobalConfig::dump, py::arg("path"));

  // glim::RawPoints
  py::class_<glim::RawPoints, std::shared_ptr<glim::RawPoints>>(m, "RawPoints", "Raw point cloud frame")
    .def(
      py::init(&create_raw_points),
      py::arg("stamp"),
      py::arg("points"),
      py::arg("times") = py::none(),
      py::arg("intensities") = py::none(),
      "Create a raw point cloud from a numpy array [N, 3] or [N, 4] with optional per-point times [N] and intensities [N]")
    .def("__repr__", [](const glim::RawPoints& p) { return "<pyglim.RawPoints stamp=" + std::to_string(p.stamp) + " size=" + std::to_string(p.size()) + ">"; })
    .def("__len__", &glim::RawPoints::size)
    .def("size", &glim::RawPoints::size)
    .def_readwrite("stamp", &glim::RawPoints::stamp, "Timestamp of the first point")
    .def_property(
      "points",
      [](const glim::RawPoints& p) { return convert_points(p.points); },
      [](glim::RawPoints& p, const DoubleArray& points) { p.points = convert_points(points); },
      "Point coordinates [N, 3]")
    .def_property(
      "times",
      [](const glim::RawPoints& p) { return py::array_t<double>(p.times.size(), p.times.data()); },
      [](glim::RawPoints& p, const DoubleArray& times) { p.times = convert_scalars(times, "times", p.size()); },
      "Per-point timestamps relative to the first point [N]")
    .def_property(
      "intensities",
      [](const glim::RawPoints& p) { return py::array_t<double>(p.intensities.size(), p.intensities.data()); },
      [](glim::RawPoints& p, const DoubleArray& intensities) { p.intensities = convert_scalars(intensities, "intensities", p.size()); },
      "Point intensities [N]");

  // glim::TimeKeeper
  py::class_<glim::TimeKeeper>(m, "TimeKeeper", "Utility class to unify timestamp convention")
    .def(py::init<>())
    .def(
      "process",
      &glim::TimeKeeper::process,
      py::arg("points"),
      "Validate and fix frame and per-point timestamps in place. Returns false if the frame should be skipped")
    .def(
      "validate_imu_stamp",
      &glim::TimeKeeper::validate_imu_stamp,
      py::arg("imu_stamp"),
      "Check if IMU and LiDAR data are roughly synchronized. Returns false if the IMU data should be skipped");

  // glim::ExtensionModule
  py::class_<glim::ExtensionModule, std::shared_ptr<glim::ExtensionModule>>(m, "ExtensionModule", "Extension module dynamically loaded from a shared library")
    .def_static("load_module", &glim::ExtensionModule::load_module, py::arg("so_name"), "Load an extension module (e.g., libstandard_viewer.so)")
    .def("ok", &glim::ExtensionModule::ok, "If false, the system should be shut down")
    .def("needs_wait", &glim::ExtensionModule::needs_wait, "Check if the module is behind the main mapping process")
    .def("at_exit", &glim::ExtensionModule::at_exit, py::arg("dump_path"), "Called when the system is quitting");
}
