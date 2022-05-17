#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glim {

enum class RegularizationMethod { NONE, PLANE, NORMALIZED_MIN_EIG, FROBENIUS };

class CloudCovarianceEstimation {
public:
  CloudCovarianceEstimation();
  ~CloudCovarianceEstimation();

  void estimate(
    const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& points,
    const std::vector<int>& neighbors,
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& normals,
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covs) const;

  void estimate(
    const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& points,
    const std::vector<int>& neighbors,
    const int k_neighbors,
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& normals,
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covs) const;

  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>
  estimate(const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& points, const std::vector<int>& neighbors, const int k_neighbors) const;

  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> estimate(
    const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& points,
    const std::vector<int>& neighbors) const;

  Eigen::Matrix4d regularize(const Eigen::Matrix4d& cov, Eigen::Vector3d* eigenvalues = nullptr, Eigen::Matrix3d* eigenvectors = nullptr) const;

private:
  RegularizationMethod regularization_method;
};

}  // namespace glim
