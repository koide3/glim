#include <glim/odometry/estimation_frame.hpp>

#include <spdlog/spdlog.h>

namespace glim {

EstimationFrame::Ptr EstimationFrame::clone() const {
  EstimationFrame::Ptr cloned(new EstimationFrame);
  *cloned = *this;
  return cloned;
}

EstimationFrame::Ptr EstimationFrame::clone_wo_points() const {
  EstimationFrame::Ptr cloned(new EstimationFrame);
  *cloned = *this;
  cloned->raw_frame.reset();
  cloned->frame.reset();
  cloned->voxelmaps.clear();
  cloned->custom_data.clear();
  return cloned;
}

const Eigen::Isometry3d EstimationFrame::T_world_sensor() const {
  switch (frame_id) {
    case FrameID::WORLD:
      return Eigen::Isometry3d::Identity();
    case FrameID::LIDAR:
      return T_world_lidar;
    case FrameID::IMU:
      return T_world_imu;
  }
  return Eigen::Isometry3d::Identity();
}

void EstimationFrame::set_T_world_sensor(FrameID frame_id, const Eigen::Isometry3d& T) {
  switch (frame_id) {
    default:
      spdlog::critical("frame_id must be either of LIDAR or IMU");
      abort();
      break;

    case FrameID::LIDAR:
      T_world_lidar = T;
      T_world_imu = T_world_lidar * T_lidar_imu;
      break;

    case FrameID::IMU:
      T_world_imu = T;
      T_world_lidar = T_world_imu * T_lidar_imu.inverse();
      break;
  }
}

}  // namespace glim