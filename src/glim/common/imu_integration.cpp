#include <glim/common/imu_integration.hpp>

#include <glim/util/config.hpp>

namespace glim {

IMUIntegrationParams::IMUIntegrationParams(const bool upright) {
  glim::Config config_sensors(glim::GlobalConfig::get_config_path("config_sensors"));

  this->upright = upright;
  this->acc_noise = config_sensors.param<double>("sensors", "imu_acc_noise", 0.01);
  this->gyro_noise = config_sensors.param<double>("sensors", "imu_gyro_noise", 0.001);
  this->int_noise = config_sensors.param<double>("sensors", "imu_int_noise", 0.001);
}

IMUIntegrationParams::~IMUIntegrationParams() {}

IMUIntegration::IMUIntegration(const IMUIntegrationParams& params) {
  auto imu_params = gtsam::PreintegrationParams::MakeSharedU();
  if (!params.upright) {
    imu_params = gtsam::PreintegrationParams::MakeSharedD();
  }

  imu_params->accelerometerCovariance = gtsam::Matrix3::Identity() * std::pow(params.acc_noise, 2);
  imu_params->gyroscopeCovariance = gtsam::Matrix3::Identity() * std::pow(params.gyro_noise, 2);
  imu_params->integrationCovariance = gtsam::Matrix3::Identity() * pow(params.int_noise, 2);
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

  if (imu_itr == imu_queue.end()) {
    return cursor;
  }

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

  const double dt = end_time - last_stamp;
  if (dt > 0.0) {
    Eigen::Matrix<double, 7, 1> last_imu_frame = imu_itr == imu_queue.end() ? *(imu_itr - 1) : *imu_itr;
    const auto& a = last_imu_frame.block<3, 1>(1, 0);
    const auto& w = last_imu_frame.block<3, 1>(4, 0);
    imu_measurements->integrateMeasurement(a, w, dt);
  }

  return cursor;
}

int IMUIntegration::integrate_imu(
  double start_time,
  double end_time,
  const gtsam::NavState& state,
  const gtsam::imuBias::ConstantBias& bias,
  std::vector<double>& pred_times,
  std::vector<Eigen::Isometry3d>& pred_poses) {
  //
  imu_measurements->resetIntegrationAndSetBias(bias);

  pred_times.emplace_back(start_time);
  pred_poses.emplace_back(Eigen::Isometry3d(state.pose().matrix()));

  int cursor = 0;
  auto imu_itr = imu_queue.begin();
  double last_stamp = start_time;

  if (imu_itr == imu_queue.end()) {
    pred_times.emplace_back(end_time);
    pred_poses.emplace_back(Eigen::Isometry3d(state.pose().matrix()));
    return cursor;
  }

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
    pred_times.emplace_back(imu_stamp);
    pred_poses.emplace_back(Eigen::Isometry3d(predicted.pose().matrix()));
    last_stamp = imu_stamp;
  }

  const double dt = end_time - last_stamp;
  if (dt > 0.0) {
    Eigen::Matrix<double, 7, 1> last_imu_frame = imu_itr == imu_queue.end() ? *(imu_itr - 1) : *imu_itr;
    const auto& a = last_imu_frame.block<3, 1>(1, 0);
    const auto& w = last_imu_frame.block<3, 1>(4, 0);
    imu_measurements->integrateMeasurement(a, w, dt);

    auto predicted = imu_measurements->predict(state, bias);
    pred_times.emplace_back(end_time);
    pred_poses.emplace_back(Eigen::Isometry3d(predicted.pose().matrix()));
  }

  return cursor;
}

int IMUIntegration::find_imu_data(double start_time, double end_time, std::vector<double>& delta_times, std::vector<Eigen::Matrix<double, 7, 1>>& imu_data) {
  //
  int cursor = 0;
  auto imu_itr = imu_queue.begin();
  double last_stamp = start_time;

  if (imu_itr == imu_queue.end()) {
    return cursor;
  }

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

    delta_times.emplace_back(dt);
    imu_data.emplace_back(imu_frame);
    last_stamp = imu_stamp;
  }

  const double dt = end_time - last_stamp;
  if (dt > 0.0) {
    Eigen::Matrix<double, 7, 1> last_imu_frame = imu_itr == imu_queue.end() ? *(imu_itr - 1) : *imu_itr;
    delta_times.emplace_back(dt);
    imu_data.emplace_back(last_imu_frame);
  }

  return cursor;
}

void IMUIntegration::erase_imu_data(int last) {
  imu_queue.erase(imu_queue.begin(), imu_queue.begin() + last);
}

const gtsam::PreintegratedImuMeasurements& IMUIntegration::integrated_measurements() const {
  return *imu_measurements;
}

const std::deque<Eigen::Matrix<double, 7, 1>>& IMUIntegration::imu_data_in_queue() const {
  return imu_queue;
}

}  // namespace glim