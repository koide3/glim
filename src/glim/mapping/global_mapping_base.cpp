#include <glim/mapping/global_mapping_base.hpp>

#include <glim/util/logging.hpp>
#include <glim/util/load_module.hpp>
#include <glim/mapping/callbacks.hpp>

namespace glim {

using Callbacks = GlobalMappingCallbacks;

GlobalMappingBase::GlobalMappingBase() : logger(create_module_logger("global")) {}

#ifdef GLIM_USE_OPENCV
void GlobalMappingBase::insert_image(const double stamp, const cv::Mat& image) {
  Callbacks::on_insert_image(stamp, image);
}
#endif

void GlobalMappingBase::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Callbacks::on_insert_imu(stamp, linear_acc, angular_vel);
}

void GlobalMappingBase::insert_submap(const SubMap::Ptr& submap) {
  Callbacks::on_insert_submap(submap);
}

void GlobalMappingBase::find_overlapping_submaps(double min_overlap) {}

void GlobalMappingBase::optimize() {}

void GlobalMappingBase::recover_graph() {}

std::shared_ptr<GlobalMappingBase> GlobalMappingBase::load_module(const std::string& so_name) {
  return load_module_from_so<GlobalMappingBase>(so_name, "create_global_mapping_module");
}
}