#include <glim/backend/callbacks.hpp>

namespace glim {

CallbackSlot<void(const double, const cv::Mat&)> SubMappingCallbacks::on_insert_image;
CallbackSlot<void(const double, const Eigen::Vector3d&, const Eigen::Vector3d&)> SubMappingCallbacks::on_insert_imu;
CallbackSlot<void(const EstimationFrame::ConstPtr& frame)> SubMappingCallbacks::on_insert_frame;

CallbackSlot<void(int id, const EstimationFrame::ConstPtr&)> SubMappingCallbacks::on_new_keyframe;

CallbackSlot<void(const SubMap::ConstPtr&)> SubMappingCallbacks::on_new_submap;
}