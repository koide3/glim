#pragma once

#include <memory>
#include <vector>
#include <Eigen/Core>

namespace glim {

class BatchedIMUMeasurements {
public:
  using Ptr = std::shared_ptr<BatchedIMUMeasurements>;
  using ConstPtr = std::shared_ptr<const BatchedIMUMeasurements>;

  BatchedIMUMeasurements(int batch_size);
  ~BatchedIMUMeasurements();

  int size() const;

  std::vector<Eigen::Matrix<double, 7, 1>>& operator[](int index);
  const std::vector<Eigen::Matrix<double, 7, 1>>& operator[](int index) const;

  void insert(int batch_index, double stamp, const Eigen::Vector3d& a, const Eigen::Vector3d& w);
  void insert(int batch_index, const Eigen::Matrix<double, 7, 1>& imu_measurement);
  void insert(int batch_index, const Eigen::Matrix<double, -1, 7>& imu_measurements);

private:
  std::vector<std::vector<Eigen::Matrix<double, 7, 1>>> imu_data;
};

}  // namespace glim