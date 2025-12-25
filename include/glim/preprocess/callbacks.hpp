#pragma once

#include <glim/util/callback_slot.hpp>
#include <glim/util/raw_points.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

namespace glim {

/**
 * @brief Point cloud preprocessing related callbacks
 *
 */
struct PreprocessCallbacks {
  /**
   * @brief Raw points arrival callback
   * @param points  Raw points received from the sensor
   */
  static CallbackSlot<void(const RawPoints::ConstPtr& points)> on_raw_points_received;

  /**
   * @brief Preprocessing start callback.
   * @note  begin -> (here) -> downsampling -> filtering & outlier removal -> end
   * @param points  Points to be preprocessed (can be modified)
   */
  static CallbackSlot<void(gtsam_points::PointCloudCPU::Ptr& points)> on_preprocessing_begin;

  /**
   * @brief Downsampling finished callback
   * @note  begin -> downsampling -> (here) -> filtering & outlier removal -> end
   * @param points  Points after downsampling (can be modified)
   */
  static CallbackSlot<void(gtsam_points::PointCloudCPU::Ptr& points)> on_downsampling_finished;

  /**
   * @brief Filtering finished callback
   * @note  begin -> downsampling -> filtering & outlier removal -> (here) -> end
   * @param points  Points after filtering (can be modified)
   */
  static CallbackSlot<void(gtsam_points::PointCloudCPU::Ptr& points)> on_filtering_finished;
};
}  // namespace glim