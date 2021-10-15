#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glim {

enum class RegularizationMethod { NONE, PLANE, FROBENIUS };

class CloudCovarianceEstimation {
public:
  CloudCovarianceEstimation();
  ~CloudCovarianceEstimation();

  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> estimate(
    const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& points,
    const std::vector<int>& neighbors) const;

  Eigen::Matrix4d regularize(const Eigen::Matrix4d& cov) const;

private:
  RegularizationMethod regularization_method;
};

}  // namespace glim
