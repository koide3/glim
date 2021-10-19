#include <glim/backend/global_mapping_base.hpp>

#include <glim/backend/callbacks.hpp>

namespace glim {

using Callbacks = GlobalMappingCallbacks;

void GlobalMappingBase::insert_image(const double stamp, const cv::Mat& image) {
  Callbacks::on_insert_image(stamp, image);
}

void GlobalMappingBase::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Callbacks::on_insert_imu(stamp, linear_acc, angular_vel);
}

void GlobalMappingBase::insert_submap(const SubMap::Ptr& submap) {
  Callbacks::on_insert_submap(submap);
}

}