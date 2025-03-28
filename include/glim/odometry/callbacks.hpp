#pragma once

#include <map>
#include <glim/util/callback_slot.hpp>
#include <glim/odometry/estimation_frame.hpp>

#ifdef GLIM_USE_OPENCV
namespace cv {
class Mat;
}
#endif

namespace gtsam {
class Values;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace gtsam_points {
class IncrementalFixedLagSmootherExt;
class IncrementalFixedLagSmootherExtWithFallback;
}  // namespace gtsam_points

namespace glim {

/**
 * @brief IMU state initialization-related callbacks
 */
struct IMUStateInitializationCallbacks {
public:
  /**
   * @brief Initialization update callback
   * @param points         Input points
   * @param T_world_lidar  Estimated LiDAR pose w.r.t. the first frame
   */
  static CallbackSlot<void(const PreprocessedFrame::ConstPtr& points, const Eigen::Isometry3d& T_odom_lidar)> on_updated;

  /**
   * @brief IMU state initialization termination callback
   * @param estimated_frame  Estimated LiDAR-IMU state
   */
  static CallbackSlot<void(const EstimationFrame::ConstPtr& estimated_frame)> on_finished;
};

/**
 * @brief Odometry estimation-related callbacks
 *
 */
struct OdometryEstimationCallbacks {
#ifdef GLIM_USE_OPENCV
  /**
   * @brief Image input callback
   * @param stamp  Timestamp
   * @param image  Image
   */
  static CallbackSlot<void(const double stamp, const cv::Mat& image)> on_insert_image;
#endif

  /**
   * @brief IMU input callback
   * @param stamp        Timestamp
   * @param linear_acc   Linear acceleration
   * @param angular_vel  Angular velocity
   */
  static CallbackSlot<void(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel)> on_insert_imu;

  /**
   * @brief PointCloud input callback
   * @param frame  Preprocessed point cloud frame
   */
  static CallbackSlot<void(const PreprocessedFrame::Ptr& frame)> on_insert_frame;

  /**
   * @brief New odometry estimation frame creation callback (Sensor state predicted with IMU forward integration)
   * @param frame  Odometry estimation frame (deskewed points and initial transformation estimate)
   */
  static CallbackSlot<void(const EstimationFrame::ConstPtr& frame)> on_new_frame;

  /**
   * @brief New odometry estimation frame update callback (Sensor state corrected with point cloud observation)
   * @param frame  Updated odometry estimation frame
   * @note  This is equivalent to `on_update_frames` with only the latest frame
   */
  static CallbackSlot<void(const EstimationFrame::ConstPtr& frame)> on_update_new_frame;

  /**
   * @brief Odometry estimation frames update callback
   * @param frames  Updated frames
   * @note  Sensor states will be updated in the odometry estimation thread
   *        Accessing them from another thread is not thread-safe
   */
  static CallbackSlot<void(const std::vector<EstimationFrame::ConstPtr>& frames)> on_update_frames;

  /**
   * @brief Odometry estimation keyframes update callback
   * @param keyframes  Updated keyframes
   * @note  Sensor states will be updated in the odometry estimation thread
   *        Accessing them from another thread is not thread-safe
   */
  static CallbackSlot<void(const std::vector<EstimationFrame::ConstPtr>& keyframes)> on_update_keyframes;

  /**
   * @brief Odometry estimation frame marginalization callback
   * @param marginalized_frames  Marginalized odometry frames
   */
  static CallbackSlot<void(const std::vector<EstimationFrame::ConstPtr>& marginalized_frames)> on_marginalized_frames;

  /**
   * @brief Odometry estimation keyframe marginalization callback
   * @param marginalized_keyframes  Marginalized odometry keyframes
   */
  static CallbackSlot<void(const std::vector<EstimationFrame::ConstPtr>& marginalized_keyframes)> on_marginalized_keyframes;

  /**
   * @brief Odometry estimation optimization callback (just before optimization)
   * @param smoother     FixedLagSmoother
   * @param new_factors  New factors to be inserted into the graph
   * @param new_values   New values to be inserted into the graph
   */
  static CallbackSlot<void(
    gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
    gtsam::NonlinearFactorGraph& new_factors,
    gtsam::Values& new_values,
    std::map<std::uint64_t, double>& new_stamps)>
    on_smoother_update;

  /**
   * @brief Odometry estimation optimization callback (just after optimization)
   * @param smoother     FixedLagSmoother
   */
  static CallbackSlot<void(gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother)> on_smoother_update_finish;

  /**
   * @brief Smoother corruption callback
   * @note  We should not forget that FixedLagSmoothers are in "gtsam_unstable" directory!!
   */
  static CallbackSlot<void(double)> on_smoother_corruption;
};

}  // namespace glim