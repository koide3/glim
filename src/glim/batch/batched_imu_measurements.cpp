#include <stdexcept>
#include <glim/batch/batched_imu_measurements.hpp>

namespace glim {

BatchedIMUMeasurements::BatchedIMUMeasurements(int batch_size) : imu_data(batch_size) {
  for (auto& data : imu_data) {
    data.reserve(100);
  }
}

BatchedIMUMeasurements::~BatchedIMUMeasurements() {}

int BatchedIMUMeasurements::size() const {
  return imu_data.size();
}

std::vector<Eigen::Matrix<double, 7, 1>>& BatchedIMUMeasurements::operator[](int index) {
  return imu_data[index];
}

const std::vector<Eigen::Matrix<double, 7, 1>>& BatchedIMUMeasurements::operator[](int index) const {
  return imu_data[index];
}

void BatchedIMUMeasurements::insert(int batch_index, double stamp, const Eigen::Vector3d& a, const Eigen::Vector3d& w) {
  insert(batch_index, (Eigen::Matrix<double, 7, 1>() << stamp, a, w).finished());
}

void BatchedIMUMeasurements::insert(int batch_index, const Eigen::Matrix<double, 7, 1>& imu_measurement) {
  if (batch_index < 0 || batch_index >= imu_data.size()) {
    throw std::out_of_range("Batch index out of range");
  }
  imu_data[batch_index].emplace_back(imu_measurement);
}

void BatchedIMUMeasurements::insert(int batch_index, const Eigen::Matrix<double, -1, 7>& imu_measurements) {
  if (batch_index < 0 || batch_index >= imu_data.size()) {
    throw std::out_of_range("Batch index out of range");
  }
  for (int i = 0; i < imu_measurements.rows(); ++i) {
    imu_data[batch_index].emplace_back(imu_measurements.row(i));
  }
}

}  // namespace glim
