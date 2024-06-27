#include <glim/odometry/odometry_estimation_ct.hpp>

extern "C" glim::OdometryEstimationBase* create_odometry_estimation_module() {
  glim::OdometryEstimationCTParams params;
  return new glim::OdometryEstimationCT(params);
}