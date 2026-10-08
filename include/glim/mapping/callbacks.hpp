#pragma once

#include <cstdint>

#include <glim/util/callback_slot.hpp>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/mapping/sub_map.hpp>

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
class ISAM2Ext;
class ISAM2ResultExt;
class LevenbergMarquardtOptimizationStatus;
}  // namespace gtsam_points

namespace glim {

enum class GraphEditState;
struct CandidateGraph;
struct SessionMergeOptions;

/**
 * @brief Sub mapping-related callbacks
 *
 */
struct SubMappingCallbacks {
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
   * @param stamp         Timestamp
   * @param linear_acc    Linear acceleration
   * @param angular_vel   Angular velocity
   */
  static CallbackSlot<void(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel)> on_insert_imu;

  /**
   * @brief Odometry estimation result input callback
   * @param frame  Marginalized odometry estimation frame
   */
  static CallbackSlot<void(const EstimationFrame::ConstPtr& frame)> on_insert_frame;

  /**
   * @brief SubMap keyframe creation callback
   * @param id        SubMap ID
   * @param keyframe  New keyframe
   */
  static CallbackSlot<void(int id, const EstimationFrame::ConstPtr& keyframe)> on_new_keyframe;

  /**
   * @brief SubMap optimization callback (just before optimization)
   * @param graph   Factor graph
   * @param values  Values to be optimized
   */
  static CallbackSlot<void(gtsam::NonlinearFactorGraph& graph, gtsam::Values& values)> on_optimize_submap;

  /**
   * @brief Optimization status callback
   * @param status  Optimization status
   * @param values  Current estimate
   */
  static CallbackSlot<void(const gtsam_points::LevenbergMarquardtOptimizationStatus& status, const gtsam::Values& values)> on_optimization_status;

  /**
   * @brief SubMap creation callback
   * @param submap  New SubMap
   */
  static CallbackSlot<void(const SubMap::ConstPtr& submap)> on_new_submap;
};

/**
 * @brief Global mapping-related callbacks
 *
 */
struct GlobalMappingCallbacks {
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
   * @brief SubMap input callback
   * @param submap  SubMap
   * @note  submap->T_world_origin is updated in the global mapping thread
   *        Accessing it from another thread is not thread-safe
   */
  static CallbackSlot<void(const SubMap::ConstPtr& submap)> on_insert_submap;

  /**
   * @brief SubMap states update callback
   * @param submaps  Updated submaps
   * @note  submap->T_world_origin are updated in the global mapping thread
   *        Accessing them from another thread is not thread-safe
   */
  static CallbackSlot<void(const std::vector<SubMap::Ptr>& submaps)> on_update_submaps;

  /**
   * @brief Global optimization callback (just before optimization)
   * @param isam2        iSAM2 Optimizer
   * @param new_factors  New factors to be inserted into the factor graph
   * @param new_values   New values to be inserted into the factor graph
   */
  static CallbackSlot<void(gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values)> on_smoother_update;

  /**
   * @brief Global optimization result callback (just after optimization)
   * @param isam2   iSAM2 optimizer
   * @param result  iSAM2 result
   */
  static CallbackSlot<void(gtsam_points::ISAM2Ext& isam2, const gtsam_points::ISAM2ResultExt& result)> on_smoother_update_result;

  /**
   * @brief Graph edit state update callback
   * @param state         Current graph edit state
   * @param pruned_mask   Submaps unavailable for selection or rendering
   */
  static CallbackSlot<void(GraphEditState state, const std::vector<uint8_t>& pruned_mask)> on_graph_edit_state_changed;

  /**
   * @brief Candidate graph view update callback
   * @param candidate Current graph being edited and displayed
   */
  static CallbackSlot<void(const CandidateGraph& candidate)> on_candidate_graph_updated;

  /**
   * @brief Request the global mapping module to perform optimization
   * @note  This is a special inverse-direction callback slot
   */
  static CallbackSlot<void()> request_to_optimize;

  /**
   * @brief Request adding factors to the graph currently accepting edits
   * @param factors Factors to add
   * @note  This is a special inverse-direction callback slot
   */
  static CallbackSlot<void(const gtsam::NonlinearFactorGraph& factors)> request_to_add_graph_factors;

  /**
   * @brief Request committing the pending session merge as a graph transaction
   * @param options Frozen session merge options
   * @note  This is a special inverse-direction callback slot
   */
  static CallbackSlot<void(const SessionMergeOptions& options)> request_to_merge_sessions;

  /**
   * @brief Request the global mapping module to detect and recover from a graph corruption
   * @note  This is a special inverse-direction callback slot
   */
  static CallbackSlot<void()> request_to_recover;

  /**
   * @brief Request the global mapping module to find new overlapping submaps
   * @param min_overlap  Minimum overlap rate between submaps
   * @note  This is a special inverse-direction callback slot
   */
  static CallbackSlot<void(double)> request_to_find_overlapping_submaps;
};
}  // namespace glim
