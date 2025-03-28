#include <glim/mapping/sub_mapping_base.hpp>

#include <glim/util/logging.hpp>
#include <glim/util/load_module.hpp>
#include <glim/mapping/callbacks.hpp>

namespace glim {

using Callbacks = SubMappingCallbacks;

SubMappingBase::SubMappingBase() : logger(create_module_logger("submap")) {}

#ifdef GLIM_USE_OPENCV
void SubMappingBase::insert_image(const double stamp, const cv::Mat& image) {
  Callbacks::on_insert_image(stamp, image);
}
#endif

void SubMappingBase::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Callbacks::on_insert_imu(stamp, linear_acc, angular_vel);
}

void SubMappingBase::insert_frame(const EstimationFrame::ConstPtr& frame) {
  Callbacks::on_insert_frame(frame);
}

std::shared_ptr<SubMappingBase> SubMappingBase::load_module(const std::string& so_name) {
  return load_module_from_so<SubMappingBase>(so_name, "create_sub_mapping_module");
}
}