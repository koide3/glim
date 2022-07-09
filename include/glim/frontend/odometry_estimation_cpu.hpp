#pragma once

#include <glim/frontend/odometry_estimation_imu.hpp>

namespace gtsam_ext {
class iVox;
class GaussianVoxelMapCPU;
}  // namespace gtsam_ext

namespace glim {

/**
 * @brief Parameters for OdometryEstimationCPU
 */
struct OdometryEstimationCPUParams : public OdometryEstimationIMUParams {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationCPUParams();
  ~OdometryEstimationCPUParams();

public:
  // Registration params
  std::string registration_type;    ///< Registration type (GICP or VGICP)
  int max_iterations;               ///< Maximum number of iterations
  int lru_thresh;                   ///< LRU cache threshold
  double target_downsampling_rate;  ///< Downsampling rate for points to be inserted into the target

  double ivox_resolution;  ///< iVox resolution (for GICP)
  double ivox_min_dist;    ///< Minimum distance between points in an iVox cell (for GICP)

  double vgicp_resolution;               ///< Voxelmap resolution (for VGICP)
  int vgicp_voxelmap_levels;             ///< Multi-resolution voxelmap levesl (for VGICP)
  double vgicp_voxelmap_scaling_factor;  ///< Multi-resolution voxelmap scaling factor (for VGICP)
};

/**
 * @brief CPU-based semi-tightly coupled LiDAR-IMU frontend
 */
class OdometryEstimationCPU : public OdometryEstimationIMU {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationCPU(const OdometryEstimationCPUParams& params = OdometryEstimationCPUParams());
  virtual ~OdometryEstimationCPU() override;

private:
  virtual gtsam::NonlinearFactorGraph create_factors(const int current, const boost::shared_ptr<gtsam::ImuFactor>& imu_factor, gtsam::Values& new_values) override;

  virtual void fallback_smoother() override;
  virtual void update_frames(const int current, const gtsam::NonlinearFactorGraph& new_factors) override;

  void update_target(const int current);

private:
  // Registration params
  std::mt19937 mt;                                                                ///< RNG
  Eigen::Isometry3d last_target_update_pose;                                      ///< Sensor post at the last target update
  std::vector<std::shared_ptr<gtsam_ext::GaussianVoxelMapCPU>> target_voxelmaps;  ///< VGICP target voxelmap
  std::shared_ptr<gtsam_ext::iVox> target_ivox;                                   ///< GICP target iVox
  EstimationFrame::ConstPtr target_ivox_frame;                                    ///< Target points (just for visualization)
};

}  // namespace glim
