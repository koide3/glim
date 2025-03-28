#include <glim/odometry/odometry_estimation_base.hpp>

#include <glim/util/logging.hpp>
#include <glim/util/load_module.hpp>
#include <glim/odometry/callbacks.hpp>

namespace glim {

using Callbacks = OdometryEstimationCallbacks;

OdometryEstimationBase::OdometryEstimationBase() : logger(create_module_logger("odom")) {}

#ifdef GLIM_USE_OPENCV
void OdometryEstimationBase::insert_image(const double stamp, const cv::Mat& image) {
  Callbacks::on_insert_image(stamp, image);
}
#endif

void OdometryEstimationBase::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Callbacks::on_insert_imu(stamp, linear_acc, angular_vel);
}

EstimationFrame::ConstPtr OdometryEstimationBase::insert_frame(const PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::ConstPtr>& marginalized_states) {
  Callbacks::on_insert_frame(frame);
  return nullptr;
}

std::shared_ptr<OdometryEstimationBase> OdometryEstimationBase::load_module(const std::string& so_name) {
  return load_module_from_so<OdometryEstimationBase>(so_name, "create_odometry_estimation_module");
}

}