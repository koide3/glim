#pragma once

#include <memory>
#include <chrono>
#include <Eigen/Core>

namespace glim {

class RawPoints;
class TimeKeeper;

class DataValidator {
public:
  DataValidator(bool debug = false);
  ~DataValidator();

public:
  void timer_callback();
  void imu_callback(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);
  void points_callback(const double stamp, const std::shared_ptr<RawPoints>& raw_points);

private:
  std::chrono::high_resolution_clock::time_point last_imu_time;
  std::chrono::high_resolution_clock::time_point last_points_time;

  double last_imu_stamp;
  double last_points_stamp;

  std::unique_ptr<TimeKeeper> time_keeper;
};

}  // namespace glim
