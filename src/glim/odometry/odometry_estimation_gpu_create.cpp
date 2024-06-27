#include <glim/odometry/odometry_estimation_gpu.hpp>

extern "C" glim::OdometryEstimationBase* create_odometry_estimation_module() {
  glim::OdometryEstimationGPUParams params;
  return new glim::OdometryEstimationGPU(params);
}