#include <pybind11/pybind11.h>

#ifdef PYGLIM_USE_CUDA
#include <gtsam_points/optimizers/linearization_hook.hpp>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu_create.hpp>
#endif

namespace py = pybind11;

void define_util(py::module_& m);
void define_preprocess(py::module_& m);
void define_odometry(py::module_& m);
void define_mapping(py::module_& m);

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

PYBIND11_MODULE(pyglim, m) {
  m.doc() = "Python bindings for GLIM (versatile and extensible LiDAR-IMU SLAM)";

#ifdef PYGLIM_USE_CUDA
  // Setup GPU-based linearization (same as glim_ros)
  gtsam_points::LinearizationHook::register_hook([]() { return gtsam_points::create_nonlinear_factor_set_gpu(); });
#endif

  define_util(m);
  define_preprocess(m);
  define_odometry(m);
  define_mapping(m);

#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif
}
