#include <glim/common/cloud_covariance_estimation.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigen>

namespace glim {

CloudCovarianceEstimation::CloudCovarianceEstimation() {
  regularization_method = RegularizationMethod::PLANE;
}

CloudCovarianceEstimation::~CloudCovarianceEstimation() {}

std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> CloudCovarianceEstimation::estimate(
  const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& points,
  const std::vector<int>& neighbors) const {
  //
  const int k_correspondences = neighbors.size() / points.size();
  assert(k_correspondences * points.size() == neighbors.size());

  // Precompute pt * pt.transpose()
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> pt_cross(points.size());
  for (int i = 0; i < points.size(); i++) {
    pt_cross[i] = points[i] * points[i].transpose();
  }

  // Calculate covariances
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> covs(points.size());
  for (int i = 0; i < points.size(); i++) {
    Eigen::Vector4d sum_points = Eigen::Vector4d::Zero();
    Eigen::Matrix4d sum_cross = Eigen::Matrix4d::Zero();

    const int begin = k_correspondences * i;
    for (int j = 0; j < k_correspondences; j++) {
      const int index = neighbors[begin + j];
      sum_points += points[index];
      sum_cross += pt_cross[index];
    }

    const Eigen::Vector4d mean = sum_points / k_correspondences;
    const Eigen::Matrix4d cov = (sum_cross - mean * sum_points.transpose()) / (k_correspondences + 1);
    covs[i] = regularize(cov);
    covs[i](3, 3) = 0.0;
  }

  return covs;
}

Eigen::Matrix4d CloudCovarianceEstimation::regularize(const Eigen::Matrix4d& cov) const {
  switch (regularization_method) {
    default:
    case RegularizationMethod::NONE:
      return cov;

    case RegularizationMethod::PLANE: {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig;
      eig.computeDirect(cov.block<3, 3>(0, 0));

      Eigen::Vector3d values(1e-3, 1.0, 1.0);
      Eigen::Matrix4d c = Eigen::Matrix4d::Zero();
      c.block<3, 3>(0, 0) = eig.eigenvectors() * values.asDiagonal() * eig.eigenvectors().inverse();
      return c;
    }

    case RegularizationMethod::FROBENIUS: {
      const double lambda = 1e-3;
      Eigen::Matrix3d C = cov.block<3, 3>(0, 0) + lambda * Eigen::Matrix3d::Identity();
      Eigen::Matrix3d C_inv = C.inverse();
      Eigen::Matrix4d C_ = Eigen::Matrix4d::Zero();
      C_.block<3, 3>(0, 0) = (C_inv / C_inv.norm()).inverse();
      return C_;
    }
  }
}

}  // namespace glim