#include <glim/common/imu_integration.hpp>

namespace glim {

IMUIntegration::IMUIntegration() {
  auto imu_params = gtsam::PreintegrationParams::MakeSharedU();
  imu_params->accelerometerCovariance = gtsam::Matrix3::Identity() * std::pow(0.005, 2);
  imu_params->gyroscopeCovariance = gtsam::Matrix3::Identity() * std::pow(0.005, 2);
  imu_params->integrationCovariance = gtsam::Matrix3::Identity() * pow(0.005, 2);
  imu_measurements.reset(new gtsam::PreintegratedImuMeasurements(imu_params));
}

IMUIntegration::~IMUIntegration() {}

void IMUIntegration::insert_imu(double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Eigen::Matrix<double, 7, 1> imu;
  imu << stamp, linear_acc, angular_vel;
  imu_queue.push_back(imu);
}

int IMUIntegration::integrate_imu(double start_time, double end_time, const gtsam::imuBias::ConstantBias& bias, int* num_integrated) {
  *num_integrated = 0;
  imu_measurements->resetIntegrationAndSetBias(bias);

  int cursor = 0;
  auto imu_itr = imu_queue.begin();
  double last_stamp = start_time;

  for (; imu_itr != imu_queue.end(); imu_itr++, cursor++) {
    const auto& imu_frame = *imu_itr;
    const double imu_stamp = imu_frame[0];

    if (imu_stamp > end_time) {
      break;
    }

    const double dt = imu_stamp - last_stamp;
    if (dt <= 0.0) {
      continue;
    }

    const auto& a = imu_frame.block<3, 1>(1, 0);
    const auto& w = imu_frame.block<3, 1>(4, 0);
    imu_measurements->integrateMeasurement(a, w, dt);

    last_stamp = imu_stamp;
    (*num_integrated)++;
  }

  return cursor;
}

int IMUIntegration::integrate_imu(
  double start_time,
  double end_time,
  const gtsam::NavState& state,
  const gtsam::imuBias::ConstantBias& bias,
  std::vector<double>& pred_times,
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& pred_poses) {
  //
  imu_measurements->resetIntegrationAndSetBias(bias);

  int cursor = 0;
  auto imu_itr = imu_queue.begin();
  double last_stamp = start_time;
  for (; imu_itr != imu_queue.end(); imu_itr++, cursor++) {
    const auto& imu_frame = *imu_itr;
    const double imu_stamp = imu_frame[0];
    if (imu_stamp > end_time) {
      break;
    }

    const double dt = imu_stamp - last_stamp;
    if (dt <= 0.0) {
      continue;
    }

    const auto& a = imu_frame.block<3, 1>(1, 0);
    const auto& w = imu_frame.block<3, 1>(4, 0);
    imu_measurements->integrateMeasurement(a, w, dt);

    auto predicted = imu_measurements->predict(state, bias);
    pred_times.push_back(imu_stamp);
    pred_poses.push_back(Eigen::Isometry3d(predicted.pose().matrix()));
    last_stamp = imu_stamp;
  }

  return cursor;
}

int IMUIntegration::integrate_imu(
  double start_time,
  double end_time,
  const gtsam::NavState& state,
  const gtsam::imuBias::ConstantBias& bias,
  std::vector<double>& pred_times,
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& pred_poses,
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& pred_vels,
  std::vector<Eigen::Matrix<double, 7, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 7, 1>>>& measurements) {
  //
  imu_measurements->resetIntegrationAndSetBias(bias);

  int cursor = 0;
  auto imu_itr = imu_queue.begin();
  double last_stamp = start_time;
  for (; imu_itr != imu_queue.end(); imu_itr++, cursor++) {
    const auto& imu_frame = *imu_itr;
    const double imu_stamp = imu_frame[0];
    if (imu_stamp > end_time) {
      break;
    }

    const double dt = imu_stamp - last_stamp;
    if (dt <= 0.0) {
      continue;
    }

    const auto& a = imu_frame.block<3, 1>(1, 0);
    const auto& w = imu_frame.block<3, 1>(4, 0);
    imu_measurements->integrateMeasurement(a, w, dt);

    auto predicted = imu_measurements->predict(state, bias);
    pred_times.push_back(imu_stamp);
    pred_poses.push_back(Eigen::Isometry3d(predicted.pose().matrix()));
    pred_vels.push_back(predicted.velocity());
    measurements.push_back(imu_frame);
    last_stamp = imu_stamp;
  }

  return cursor;
}

void IMUIntegration::erase_imu_data(int last) {
  imu_queue.erase(imu_queue.begin(), imu_queue.begin() + last);
}

const gtsam::PreintegratedImuMeasurements& IMUIntegration::integrated_measurements() const {
  return *imu_measurements;
}

}  // namespace glim