#include <glim/util/time_keeper.hpp>

#include <spdlog/spdlog.h>
#include <boost/format.hpp>
#include <glim/util/console_colors.hpp>

namespace glim {

TimeKeeper::TimeKeeper(const AbsPointTimeParams& abs_params) : abs_params(abs_params) {
  first_warning = true;
  last_points_stamp = -1.0;
  last_imu_stamp = -1.0;

  num_scans = 0;
  first_points_stamp = 0.0;
  estimated_scan_duration = -1.0;
  point_time_offset = 0.0;
}

TimeKeeper::~TimeKeeper() {}

void TimeKeeper::validate_imu_stamp(const double imu_stamp) {
  const double imu_diff = imu_stamp - last_imu_stamp;
  if (last_imu_stamp < 0.0) {
    // First IMU frame
  } else if (imu_stamp < last_imu_stamp) {
    spdlog::warn("IMU IMU timestamp rewind detected!!");
    spdlog::warn("current={:.6f} last={:.6f} diff={:.6f}", imu_stamp, last_imu_stamp, imu_diff);
  } else if (imu_stamp - last_imu_stamp > 0.1) {
    spdlog::warn("large time gap between consecutive IMU data!!");
    spdlog::warn("current={:.6f} last={:.6f} diff={:.6f}", imu_stamp, last_imu_stamp, imu_diff);
  }
  last_imu_stamp = imu_stamp;

  const double points_diff = imu_stamp - last_points_stamp;
  if (last_points_stamp > 0.0 && std::abs(points_diff) > 1.0) {
    spdlog::warn("large time difference between points and imu!!");
    spdlog::warn("points={:.6f} imu={:.6f} diff={:.6f}", last_points_stamp, imu_stamp, points_diff);
  }
}

void TimeKeeper::process(const glim::RawPoints::Ptr& points) {
  replace_points_stamp(points);

  const double time_diff = points->stamp - last_points_stamp;
  if (last_points_stamp < 0.0) {
    // First LiDAR frame
  } else if (time_diff < 0.0) {
    spdlog::warn("point timestamp rewind detected!!");
    spdlog::warn("current={:.6f} last={:.6f} diff={:.6f}", points->stamp, last_points_stamp, time_diff);
  } else if (time_diff > 0.5) {
    spdlog::warn("large time gap between consecutive LiDAR frames!!");
    spdlog::warn("current={:.6f} last={:.6f} diff={:.6f}", points->stamp, last_points_stamp, time_diff);
  }

  last_points_stamp = points->stamp;
}

void TimeKeeper::replace_points_stamp(const glim::RawPoints::Ptr& points) {
  // No per-point timestamps
  // Assign timestamps based on scan duration
  if (points->times.empty()) {
    if (first_warning) {
      spdlog::warn("per-point timestamps are not given!!");
      spdlog::warn("use pseudo per-point timestamps based on the order of points");
      first_warning = false;
    }

    points->times.resize(points->size(), 0.0);
    const double scan_duration = estimate_scan_duration(points->stamp);
    if (scan_duration > 0.0) {
      for (int i = 0; i < points->size(); i++) {
        points->times[i] = scan_duration * static_cast<double>(i) / points->size();
      }
    }

    return;
  }

  // Check the number of timestamps
  if (points->times.size() != points->size()) {
    spdlog::warn("# of timestamps and # of points mismatch!!");
    points->times.resize(points->size(), 0.0);
    return;
  }

  // Check if the per-point timestamps are positive
  if (points->times.front() < 0.0 || points->times.back() < 0.0) {
    spdlog::warn("negative per-point timestamp is found!!");
    spdlog::warn("front={:.6f} back={:.6f}", points->times.front(), points->times.back());
  }

  // Point timestamps are already relative to the first one
  if (points->times.front() < 1.0) {
    first_warning = false;
    return;
  }

  if (first_warning) {
    spdlog::warn("large point timestamp ({:.6f} > 1.0) found!!", points->times.back());
    spdlog::warn("assume that point times are absolute and convert them to relative");
    spdlog::warn("replace_frame_stamp={} wrt_first_frame_timestamp={}", abs_params.replace_frame_timestamp, abs_params.wrt_first_frame_timestamp);
  }

  // Convert absolute times to relative times
  if (abs_params.replace_frame_timestamp) {
    if (!abs_params.wrt_first_frame_timestamp || std::abs(points->stamp - points->times.front()) < 1.0) {
      if (first_warning) {
        spdlog::warn("use first point timestamp as frame timestamp");
        spdlog::warn("frame={:.6f} point={:.6f}", points->stamp, points->times.front());
      }

      point_time_offset = 0.0;
      points->stamp = points->times.front();
    } else {
      if (first_warning) {
        spdlog::warn("point timestamp is too apart from frame timestamp!!");
        spdlog::warn("use time offset w.r.t. the first frame timestamp");
        spdlog::warn("frame={:.6f} point={:.6f} diff={:.6f}", points->stamp, points->times.front(), points->stamp - points->times.front());

        point_time_offset = points->stamp - points->times.front();
      }

      points->stamp = points->times.front() + point_time_offset;
    }

    const double first_stamp = points->times.front();
    for (auto& t : points->times) {
      t -= first_stamp;
    }
  }

  first_warning = false;
}

double TimeKeeper::estimate_scan_duration(const double stamp) {
  if (estimated_scan_duration > 0.0) {
    return estimated_scan_duration;
  }

  if ((num_scans++) == 0) {
    first_points_stamp = stamp;
    return -1.0;
  }

  const double scan_duration = (stamp - first_points_stamp) / (num_scans - 1);

  if (num_scans == 1000) {
    spdlog::info("estimated scan duration: {}", scan_duration);
    estimated_scan_duration = scan_duration;
  }

  return scan_duration;
}
}  // namespace glim