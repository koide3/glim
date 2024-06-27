#include <glim/odometry/odometry_estimation_cpu.hpp>

extern "C" glim::OdometryEstimationBase* create_odometry_estimation_module() {
  glim::OdometryEstimationCPUParams params;
  return new glim::OdometryEstimationCPU(params);
}