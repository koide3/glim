#include <glim/odometry/odometry_estimation_custom.hpp>

extern "C" glim::OdometryEstimationBase* create_odometry_estimation_module() {
  auto params = std::make_unique<glim::OdometryEstimationCustomParams>();
  return new glim::OdometryEstimationCustom(std::move(params));
}