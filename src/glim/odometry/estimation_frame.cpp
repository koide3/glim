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
  return cloned;
}

const Eigen::Isometry3d EstimationFrame::T_world_sensor() const {
  switch (frame_id) {
    case FrameID::WORLD:
      return Eigen::Isometry3d::Identity();
    case FrameID::BASE:
      return T_world_base;
    case FrameID::IMU:
      return T_world_imu;
    case FrameID::GNSS:
      return T_world_gnss;
    case FrameID::LIDAR:
      return T_world_lidar;
  }
  return Eigen::Isometry3d::Identity();
}

void EstimationFrame::set_T_world_sensor(FrameID frame_id, const Eigen::Isometry3d& T) {
  switch (frame_id) {
    default:
      spdlog::critical("frame_id must be either of BASE, IMU, GNSS, or LIDAR");
      abort();
      break;

    case FrameID::BASE:
      T_world_base = T;
      T_world_imu = T_world_base * T_base_imu;
      T_world_lidar = T_world_base * T_base_lidar;
      T_world_gnss = T_world_base * T_base_gnss;
      break;

    case FrameID::IMU:
      T_world_imu = T;
      T_world_base = T * T_base_imu.inverse();
      T_world_gnss = T_world_base * T_base_gnss;
      T_world_lidar = T_world_base * T_base_lidar;
      break;

    case FrameID::GNSS:
      T_world_gnss = T;
      T_world_base = T * T_base_gnss.inverse();
      T_world_imu = T_world_base * T_base_imu;
      T_world_lidar = T_world_base * T_base_lidar;
      break;

    case FrameID::LIDAR:
      T_world_lidar = T;
      T_world_base = T * T_base_lidar.inverse();
      T_world_imu = T_world_base * T_base_imu;
      T_world_gnss = T_world_base * T_base_gnss;
      break;
  }
}

}  // namespace glim