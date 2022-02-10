#pragma once

#include <deque>
#include <glim/util/raw_points.hpp>

namespace glim {

struct AbsPointTimeParams {
public:
  AbsPointTimeParams() {
    replace_frame_timestamp = true;
    wrt_first_frame_timestamp = true;
  }

  bool replace_frame_timestamp;
  bool wrt_first_frame_timestamp;
};

/**
 * @brief Utility class to unify timestamp convension
 */
class TimeKeeper {
public:
  TimeKeeper(const AbsPointTimeParams& abs_params = AbsPointTimeParams());
  ~TimeKeeper();

  /**
   * @brief Replace frame and point timestamps
   * @note  Frame timestamp must be the one at the moment when the first point is acquired
   * @note  Point timestamps must be relative with respect to the first point
   */
  void process(const glim::RawPoints::Ptr& points);

  /**
   * @brief Check if IMU and LiDAR data are (very roughly) synchronized
   */
  void validate_imu_stamp(const double imu_stamp);

private:
  double estimate_scan_duration(const double stamp);

private:
  const AbsPointTimeParams abs_params;

  bool first_warning;        // Flag to show warning messages only once
  double last_points_stamp;  // Timestamp of last frame

  int num_scans;                   // Number of frames for scan duration estimation
  double first_points_stamp;       // Timestamp of the first frame for scan duration estimation
  double estimated_scan_duration;  // Estimated scan duration

  double point_time_offset;  // Offset to correct time shift of point times
};

}  // namespace glim