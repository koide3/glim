#include <glim/backend/callbacks.hpp>

namespace glim {

CallbackSlot<void(const double, const cv::Mat&)> SubMappingCallbacks::on_insert_image;
CallbackSlot<void(const double, const Eigen::Vector3d&, const Eigen::Vector3d&)> SubMappingCallbacks::on_insert_imu;
CallbackSlot<void(const EstimationFrame::ConstPtr& frame)> SubMappingCallbacks::on_insert_frame;
CallbackSlot<void(int id, const EstimationFrame::ConstPtr&)> SubMappingCallbacks::on_new_keyframe;
CallbackSlot<void(const SubMap::ConstPtr&)> SubMappingCallbacks::on_new_submap;

CallbackSlot<void(const double, const cv::Mat&)> GlobalMappingCallbacks::on_insert_image;
CallbackSlot<void(const double, const Eigen::Vector3d&, const Eigen::Vector3d&)> GlobalMappingCallbacks::on_insert_imu;
CallbackSlot<void(const SubMap::ConstPtr& frame)> GlobalMappingCallbacks::on_insert_submap;

CallbackSlot<void(const std::vector<SubMap::Ptr>& submaps)> GlobalMappingCallbacks::on_update_submaps;

CallbackSlot<void(gtsam_ext::ISAM2Ext&, gtsam::NonlinearFactorGraph&, gtsam::Values&)> GlobalMappingCallbacks::on_smoother_update;
CallbackSlot<void(gtsam_ext::ISAM2Ext&, const gtsam_ext::ISAM2ResultExt& result)> GlobalMappingCallbacks::on_smoother_update_result;
}  // namespace glim