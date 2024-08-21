#include <glim/util/time_keeper.hpp>

#include <optional>
#include <spdlog/spdlog.h>
#include <boost/format.hpp>
#include <glim/util/config.hpp>

namespace glim {

PerPointTimeSettings::PerPointTimeSettings() {
  const Config config(GlobalConfig::get_config_path("config_sensors"));
  autoconf = config.param<bool>("sensors", "autoconf_perpoint_times", true);

  if (autoconf) {
    relative_time = true;
    prefer_frame_time = false;
    point_time_scale = 1.0;
  } else {
    relative_time = config.param<bool>("sensors", "perpoint_relative_time", true);
    prefer_frame_time = config.param<bool>("sensors", "autoconf_prefer_frame_time", false);
    point_time_scale = config.param<double>("sensors", "perpoint_time_scale", 1.0);
  }
}

PerPointTimeSettings::~PerPointTimeSettings() {}

TimeKeeper::TimeKeeper() {
  last_points_stamp = -1.0;
  last_imu_stamp = -1.0;

  estimated_scan_duration = -1.0;
  point_time_offset = 0.0;
}

TimeKeeper::~TimeKeeper() {}

bool TimeKeeper::validate_imu_stamp(const double imu_stamp) {
  const double imu_diff = imu_stamp - last_imu_stamp;
  if (last_imu_stamp < 0.0) {
    // First IMU frame
  } else if (imu_stamp < last_imu_stamp) {
    spdlog::warn("IMU IMU timestamp rewind detected!!");
    spdlog::warn("current={:.6f} last={:.6f} diff={:.6f}", imu_stamp, last_imu_stamp, imu_diff);
    return false;
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

  return true;
}

void TimeKeeper::process(const glim::RawPoints::Ptr& points) {
  replace_points_stamp(points);

  if (points->points.size() != points->times.size()) {
    // Here must not be reached
    spdlog::error("inconsistent # of points and # of timestamps found after time conversion!! |points|={} |times|={}", points->points.size(), points->times.size());
  }
  if (points->times.front() < 0.0 || points->times.back() < 0.0) {
    // Here must not be reached
    spdlog::error("negative per-point timestamp is found after time conversion!! front={:.6f} back={:.6f}", points->times.front(), points->times.back());
  }
  if (points->times.front() > 1.0 || points->times.back() > 1.0) {
    // Here must not be reached
    spdlog::error("large per-point timestamp is found after time conversion!! front={:.6f} back={:.6f}", points->times.front(), points->times.back());
  }
  if (points->stamp < 0.0) {
    spdlog::warn("frame timestamp is negative!! frame={:.6f}", points->stamp);
  }
  if (points->stamp > 3000000000) {
    spdlog::warn("frame timestamp is wrong (or GLIM has been used for over 40 years)!! frame={:.6f}", points->stamp);
  }

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
  // Assign timestamps based on the estimated scan duration
  if (points->times.empty()) {
    static bool first_warning = true;
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

  const auto minmax_times = std::minmax_element(points->times.begin(), points->times.end());
  const double min_time = *minmax_times.first;
  const double max_time = *minmax_times.second;

  if (settings.autoconf) {
    settings.autoconf = false;

    if (min_time < 0.0) {
      spdlog::warn("negative per-point timestamp is found!! min={:.6f} max={:.6f}", min_time, max_time);

      if (settings.prefer_frame_time) {
        spdlog::warn("use frame timestamp as is!!");
      } else {
        spdlog::warn("add an offset to the frame timestamp to make per-point ones positive!!");
      }
    }

    if (max_time < 1.0) {
      settings.relative_time = true;
    } else {
      settings.relative_time = false;
      spdlog::warn("large point timestamp (min={:.6f} max={:.6f} > 1.0) found!!", min_time, max_time);
      spdlog::warn("assume that point times are absolute and convert them to relative");

      if (min_time > 1e16) {
        spdlog::warn("too large point timestamp (min={:.6f} max={:.6f} > 1e16) found!!", min_time, max_time);
        spdlog::warn("maybe using a Livox LiDAR that use FLOAT64 nanosec per-point timestamps");
        settings.point_time_scale = 1e-9;
      }

      if (settings.prefer_frame_time) {
        spdlog::warn("frame timestamp will be prioritized over the first point timestamp!!");
      } else {
        spdlog::warn("frame timestamp will be overwritten by the first point timestamp!!");
      }
    }
  }

  // Per-point timestamps are relative to the first one
  if (settings.relative_time) {
    // Make per-point timestamps positive
    if (min_time < 0.0) {
      if (!settings.prefer_frame_time) {
        // Shift the frame timestamp to keep the consistency
        points->stamp += min_time * settings.point_time_scale;
      }

      for (auto& time : points->times) {
        time -= min_time;
      }
    }

    if (std::abs(settings.point_time_scale - 1.0) > 1e-6) {
      // Convert timestamps to seconds
      for (auto& time : points->times) {
        time *= settings.point_time_scale;
      }
    }

    return;
  }

  // Per-point timestamps are absolute

  if (!settings.prefer_frame_time) {
    // Overwrite the frame timestamp with the first point timestamp
    points->stamp = min_time * settings.point_time_scale;
  }

  // Make per-point timestamps relative to the frame timestamp
  for (auto& time : points->times) {
    time = (time - min_time) * settings.point_time_scale;
  }
}

double TimeKeeper::estimate_scan_duration(const double stamp) {
  if (estimated_scan_duration > 0.0) {
    return estimated_scan_duration;
  }

  if (last_points_stamp < 0) {
    return -1.0;
  }

  scan_duration_history.emplace_back(stamp - last_points_stamp);
  std::nth_element(scan_duration_history.begin(), scan_duration_history.begin() + scan_duration_history.size() / 2, scan_duration_history.end());
  double scan_duration = scan_duration_history[scan_duration_history.size() / 2];

  if (scan_duration_history.size() == 1000) {
    spdlog::info("estimated scan duration: {}", scan_duration);
    estimated_scan_duration = scan_duration;
    scan_duration_history.clear();
    scan_duration_history.shrink_to_fit();
  }

  if (scan_duration < 0.01 || scan_duration > 1.0) {
    spdlog::warn("invalid scan duration estimate: {}", scan_duration);
    scan_duration = -1.0;
  }

  return scan_duration;
}
}  // namespace glim