#include <glim/odometry/callbacks.hpp>

#include <gtsam_unstable/nonlinear/FixedLagSmoother.h>

namespace glim {

CallbackSlot<void(const PreprocessedFrame::ConstPtr& points, const Eigen::Isometry3d& T_odom_lidar)> IMUStateInitializationCallbacks::on_updated;
CallbackSlot<void(const EstimationFrame::ConstPtr& estimated_frame)> IMUStateInitializationCallbacks::on_finished;

#ifdef GLIM_USE_OPENCV
CallbackSlot<void(const double, const cv::Mat&)> OdometryEstimationCallbacks::on_insert_image;
#endif
CallbackSlot<void(const double, const Eigen::Vector3d&, const Eigen::Vector3d&)> OdometryEstimationCallbacks::on_insert_imu;
CallbackSlot<void(const PreprocessedFrame::Ptr& frame)> OdometryEstimationCallbacks::on_insert_frame;

CallbackSlot<void(const EstimationFrame::ConstPtr&)> OdometryEstimationCallbacks::on_new_frame;
CallbackSlot<void(const EstimationFrame::ConstPtr&)> OdometryEstimationCallbacks::on_update_new_frame;

CallbackSlot<void(const std::vector<EstimationFrame::ConstPtr>&)> OdometryEstimationCallbacks::on_marginalized_frames;
CallbackSlot<void(const std::vector<EstimationFrame::ConstPtr>&)> OdometryEstimationCallbacks::on_marginalized_keyframes;

CallbackSlot<void(const std::vector<EstimationFrame::ConstPtr>&)> OdometryEstimationCallbacks::on_update_frames;
CallbackSlot<void(const std::vector<EstimationFrame::ConstPtr>&)> OdometryEstimationCallbacks::on_update_keyframes;

CallbackSlot<void(gtsam_points::IncrementalFixedLagSmootherExtWithFallback&, gtsam::NonlinearFactorGraph&, gtsam::Values&, gtsam::FixedLagSmootherKeyTimestampMap&)>
  OdometryEstimationCallbacks::on_smoother_update;

CallbackSlot<void(gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother)> OdometryEstimationCallbacks::on_smoother_update_finish;

CallbackSlot<void(double)> OdometryEstimationCallbacks::on_smoother_corruption;

}  // namespace glim