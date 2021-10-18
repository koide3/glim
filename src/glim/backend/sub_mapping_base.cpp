#include <glim/backend/sub_mapping_base.hpp>

#include <glim/backend/callbacks.hpp>

namespace glim {

using Callbacks = SubMappingCallbacks;

void SubMappingBase::insert_image(const double stamp, const cv::Mat& image) {
  Callbacks::on_insert_image(stamp, image);
}

void SubMappingBase::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Callbacks::on_insert_imu(stamp, linear_acc, angular_vel);
}

void SubMappingBase::insert_frame(const EstimationFrame::ConstPtr& frame) {
  Callbacks::on_insert_frame(frame);
}

}