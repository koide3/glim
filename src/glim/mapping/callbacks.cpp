#include <glim/mapping/callbacks.hpp>

namespace glim {

// sub mapping
#ifdef GLIM_USE_OPENCV
CallbackSlot<void(const double, const cv::Mat&)> SubMappingCallbacks::on_insert_image;
#endif
CallbackSlot<void(const double, const Eigen::Vector3d&, const Eigen::Vector3d&)> SubMappingCallbacks::on_insert_imu;
CallbackSlot<void(const EstimationFrame::ConstPtr& frame)> SubMappingCallbacks::on_insert_frame;
CallbackSlot<void(int id, const EstimationFrame::ConstPtr&)> SubMappingCallbacks::on_new_keyframe;

CallbackSlot<void(gtsam::NonlinearFactorGraph&, gtsam::Values&)> SubMappingCallbacks::on_optimize_submap;
CallbackSlot<void(const gtsam_points::LevenbergMarquardtOptimizationStatus&, const gtsam::Values& values)> SubMappingCallbacks::on_optimization_status;
CallbackSlot<void(const SubMap::ConstPtr&)> SubMappingCallbacks::on_new_submap;

// global mapping
#ifdef GLIM_USE_OPENCV
CallbackSlot<void(const double, const cv::Mat&)> GlobalMappingCallbacks::on_insert_image;
#endif
CallbackSlot<void(const double, const Eigen::Vector3d&, const Eigen::Vector3d&)> GlobalMappingCallbacks::on_insert_imu;
CallbackSlot<void(const SubMap::ConstPtr& frame)> GlobalMappingCallbacks::on_insert_submap;

CallbackSlot<void(const std::vector<SubMap::Ptr>& submaps)> GlobalMappingCallbacks::on_update_submaps;

CallbackSlot<void(gtsam_points::ISAM2Ext&, gtsam::NonlinearFactorGraph&, gtsam::Values&)> GlobalMappingCallbacks::on_smoother_update;
CallbackSlot<void(gtsam_points::ISAM2Ext&, const gtsam_points::ISAM2ResultExt& result)> GlobalMappingCallbacks::on_smoother_update_result;

CallbackSlot<void()> GlobalMappingCallbacks::request_to_optimize;
CallbackSlot<void()> GlobalMappingCallbacks::request_to_recover;
CallbackSlot<void(double)> GlobalMappingCallbacks::request_to_find_overlapping_submaps;
}  // namespace glim