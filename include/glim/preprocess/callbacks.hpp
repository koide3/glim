#pragma once

#include <glim/util/callback_slot.hpp>
#include <glim/util/raw_points.hpp>

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
};
}