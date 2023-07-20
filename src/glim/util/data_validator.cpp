#include <glim/util/data_validator.hpp>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/dup_filter_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <glim/util/time_keeper.hpp>
#include <glim/util/raw_points.hpp>
#include <glim/util/convert_to_string.hpp>

namespace glim {

DataValidator::DataValidator(bool debug) {
  auto dup_filter = std::make_shared<spdlog::sinks::dup_filter_sink_mt>(std::chrono::seconds(10));
  dup_filter->add_sink(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
  auto logger = std::make_shared<spdlog::logger>("validator", dup_filter);

  if (debug) {
    logger->set_level(spdlog::level::debug);
  }

  spdlog::set_default_logger(logger);

  spdlog::info("Starting data validator");

  last_imu_time = std::chrono::high_resolution_clock::now();
  last_points_time = std::chrono::high_resolution_clock::now();

  last_imu_stamp = -1.0;
  last_points_stamp = -1.0;

  time_keeper.reset(new TimeKeeper());
}

DataValidator::~DataValidator() {}

void DataValidator::timer_callback() {
  const auto t1 = std::chrono::high_resolution_clock::now();
  spdlog::debug("timer_callback");

  const auto now = std::chrono::high_resolution_clock::now();
  if (now - last_imu_time > std::chrono::seconds(1)) {
    spdlog::warn("No IMU data received in last 1 second");
  }
  if (now - last_points_time > std::chrono::seconds(1)) {
    spdlog::warn("No points received in last 1 second");
  }

  spdlog::debug("timer_callback done (elapsed={:.3f}[msec])", std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - t1).count() / 1e6);
}

void DataValidator::imu_callback(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  spdlog::trace("imu_callback (stamp={:.6f})", stamp);

  last_imu_time = std::chrono::high_resolution_clock::now();

  if (last_imu_stamp >= 0.0) {
    if (stamp < last_imu_stamp) {
      spdlog::warn("IMU timestamp rewind detected!! (last={:.6f} current={:.6f})", last_imu_stamp, stamp);
    } else if (stamp - last_imu_stamp > 0.1) {
      spdlog::warn("A large time gap between consecutive IMU data!! (last={:.6f} current={:.6f} diff={:.6f})", last_imu_stamp, stamp, stamp - last_imu_stamp);
    }
  }

  last_imu_stamp = stamp;
}

void DataValidator::points_callback(const double stamp, const std::shared_ptr<RawPoints>& raw_points) {
  const auto t1 = std::chrono::high_resolution_clock::now();
  spdlog::debug("points_callback (stamp={:.6f})", stamp);

  last_points_time = std::chrono::high_resolution_clock::now();

  if (last_points_stamp >= 0.0) {
    if (stamp < last_points_stamp) {
      spdlog::warn("Point cloud timestamp rewind detected!! (last={:.6f} current={:.6f})", last_points_stamp, stamp);
    } else if (stamp - last_points_stamp > 1.0) {
      spdlog::warn("A large time gap between consecutive point clouds!! (last={:.6f} current={:.6f} diff={:.6f})", last_points_stamp, stamp, stamp - last_points_stamp);
    }
  }

  if (last_imu_stamp >= 0.0) {
    if (std::abs(last_imu_stamp - stamp) > 1.0) {
      spdlog::warn("Too large time gap between points and IMU timestamps!! (IMU={:.6f} points={:.6f})", last_imu_stamp, stamp);
    }
  }

  time_keeper->process(raw_points);

  spdlog::debug("points size={} stamp={:.6f} times={:.3f}~{:.3f}", raw_points->size(), raw_points->stamp, raw_points->times[0], raw_points->times[raw_points->size() - 1]);

  if (std::any_of(raw_points->times.begin(), raw_points->times.end(), [](double t) { return t < 0.0; })) {
    spdlog::warn("A negative per-point timestamp found!! (t={:.6f})", *std::min_element(raw_points->times.begin(), raw_points->times.end()));
  }

  if (std::any_of(raw_points->times.begin(), raw_points->times.end(), [](double t) { return t > 1.0; })) {
    spdlog::warn("A large per-point timestamp found!! (t={:.6f})", *std::max_element(raw_points->times.begin(), raw_points->times.end()));
  }

  for (int i = 0; i < raw_points->size(); i++) {
    if (!raw_points->points[i].array().isFinite().all()) {
      spdlog::warn("An invalid point found!! (pt={})", convert_to_string(raw_points->points[i]));
      break;
    }
  }

  last_points_stamp = stamp;

  spdlog::debug("points_callback done (elapsed={:.3f}[msec])", std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - t1).count() / 1e6);
}

}  // namespace glim