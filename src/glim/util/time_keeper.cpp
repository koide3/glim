#include <glim/util/time_keeper.hpp>

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
    std::cerr << console::yellow << "warning: IMU timestamp rewind detected!!" << console::reset << std::endl;
    std::cerr << console::yellow << boost::format("       : current:%.6f last:%.6f diff:%.6f") % imu_stamp % last_imu_stamp % imu_diff << console::reset << std::endl;
  } else if (imu_stamp - last_imu_stamp > 0.1) {
    std::cerr << console::yellow << "warning: large time gap between consecutive IMU data!!" << console::reset << std::endl;
    std::cerr << console::yellow << boost::format("       : current:%.6f last:%.6f diff:%.6f") % imu_stamp % last_imu_stamp % imu_diff << console::reset << std::endl;
  }
  last_imu_stamp = imu_stamp;

  const double points_diff = imu_stamp - last_points_stamp;
  if (last_points_stamp > 0.0 && std::abs(points_diff) > 1.0) {
    std::cerr << console::yellow << "warning: large time difference between points and imu!!" << console::reset << std::endl;
    std::cerr << console::yellow << boost::format("       : points:%.6f imu:%.6f diff:%.6f") % last_points_stamp % imu_stamp % points_diff << console::reset << std::endl;
  }
}

void TimeKeeper::process(const glim::RawPoints::Ptr& points) {
  replace_points_stamp(points);

  const double time_diff = points->stamp - last_points_stamp;
  if (last_points_stamp < 0.0) {
    // First LiDAR frame
  } else if (time_diff < 0.0) {
    std::cerr << console::yellow << "warning: point timestamp rewind detected!!" << console::reset << std::endl;
    std::cerr << console::yellow << boost::format("       : current:%.6f last:%.6f diff:%.6f") % points->stamp % last_points_stamp % time_diff << console::reset << std::endl;
  } else if (time_diff > 0.5) {
    std::cerr << console::yellow << "warning: large time gap between consecutive LiDAR frames!!" << console::reset << std::endl;
    std::cerr << console::yellow << boost::format("       : current:%.6f last:%.6f diff:%.6f") % points->stamp % last_points_stamp % time_diff << console::reset << std::endl;
  }

  last_points_stamp = points->stamp;
}

void TimeKeeper::replace_points_stamp(const glim::RawPoints::Ptr& points) {
  // No per-point timestamps
  // Assign timestamps based on scan duration
  if (points->times.empty()) {
    if (first_warning) {
      std::cerr << console::yellow << boost::format("warning: per-point timestamps are not given!!") << console::reset << std::endl;
      std::cerr << console::yellow << boost::format("       : use pseudo per-point timestamps based on the order of points") << console::reset << std::endl;
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
    std::cerr << console::yellow << "warning: # of timestamps and # of points mismatch!!" << console::reset << std::endl;
    points->times.resize(points->size(), 0.0);
    return;
  }

  // Point timestamps are already relative to the first one
  if (points->times.front() < 1.0) {
    return;
  }

  if (first_warning) {
    std::cerr << console::yellow << boost::format("warning: large point timestamp (%.6f > 1.0) found!!") % points->times.back() << console::reset << std::endl;
    std::cerr << console::yellow << boost::format("       : assume that point times are absolute and convert them to relative") << console::reset << std::endl;
    std::cerr << console::yellow
              << boost::format("       : replace_frame_stamp=%d wrt_first_frame_timestamp=%d") % abs_params.replace_frame_timestamp % abs_params.wrt_first_frame_timestamp
              << console::reset << std::endl;
  }

  // Convert absolute times to relative times
  if (abs_params.replace_frame_timestamp) {
    if (!abs_params.wrt_first_frame_timestamp || std::abs(points->stamp - points->times.front()) < 1.0) {
      if (first_warning) {
        std::cerr << console::yellow << boost::format("warning: use first point timestamp as frame timestamp") << console::reset << std::endl;
        std::cerr << console::yellow << boost::format("       : frame=%.6f point=%.6f") % points->stamp % points->times.front() << console::reset << std::endl;
      }

      point_time_offset = 0.0;
      points->stamp = points->times.front();
    } else {
      if (first_warning) {
        std::cerr << console::yellow << boost::format("warning: point timestamp is too apart from frame timestamp!!") << console::reset << std::endl;
        std::cerr << console::yellow << boost::format("       : use time offset w.r.t. the first frame timestamp") << console::reset << std::endl;
        std::cerr << console::yellow << boost::format("       : frame=%.6f point=%.6f diff=%.6f") % points->stamp % points->times.front() % (points->stamp - points->times.front())
                  << console::reset << std::endl;

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
    std::cerr << console::yellow << "estimated scan duration:" << scan_duration << console::reset << std::endl;
    estimated_scan_duration = scan_duration;
  }

  return scan_duration;
}
}  // namespace glim