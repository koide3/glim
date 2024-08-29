#include <glim/common/cloud_covariance_estimation.hpp>

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigen>

#include <spdlog/spdlog.h>

namespace glim {

CloudCovarianceEstimation::CloudCovarianceEstimation(const int num_threads, const double plane_eps)
: regularization_method(RegularizationMethod::PLANE),
  num_threads(num_threads),
  plane_eps(plane_eps) {}

CloudCovarianceEstimation::~CloudCovarianceEstimation() {}

void CloudCovarianceEstimation::estimate(
  const std::vector<Eigen::Vector4d>& points,
  const std::vector<int>& neighbors,
  std::vector<Eigen::Vector4d>& normals,
  std::vector<Eigen::Matrix4d>& covs) const {
  //
  if (points.empty()) {
    return;
  }

  const int k = neighbors.size() / points.size();
  if (k * points.size() != neighbors.size()) {
    spdlog::critical("k * points.size() != neighbors.size()");
    abort();
  }

  estimate(points, neighbors, k, normals, covs);
}

void CloudCovarianceEstimation::estimate(
  const std::vector<Eigen::Vector4d>& points,
  const std::vector<int>& neighbors,
  const int k_neighbors,
  std::vector<Eigen::Vector4d>& normals,
  std::vector<Eigen::Matrix4d>& covs) const {
  if (points.empty()) {
    return;
  }

  const int k_correspondences = neighbors.size() / points.size();
  assert(k_correspondences * points.size() == neighbors.size());
  assert(k_neighbors <= k_correspondences);

  std::vector<Eigen::Matrix4d> pt_cross(points.size());
#pragma omp parallel for num_threads(num_threads) schedule(guided, 8)
  for (int i = 0; i < points.size(); i++) {
    pt_cross[i] = points[i] * points[i].transpose();
  }

  normals.resize(points.size());
  covs.resize(points.size());

  // Calculate covariances
#pragma omp parallel for num_threads(num_threads) schedule(guided, 8)
  for (int i = 0; i < points.size(); i++) {
    Eigen::Vector4d sum_points = Eigen::Vector4d::Zero();
    Eigen::Matrix4d sum_cross = Eigen::Matrix4d::Zero();

    const int begin = k_correspondences * i;
    for (int j = 0; j < k_neighbors; j++) {
      const int index = neighbors[begin + j];
      sum_points += points[index];
      sum_cross += pt_cross[index];
    }

    const Eigen::Vector4d mean = sum_points / k_neighbors;
    const Eigen::Matrix4d cov = (sum_cross - mean * sum_points.transpose()) / k_neighbors;

    Eigen::Matrix3d eigenvectors;
    covs[i] = regularize(cov, nullptr, &eigenvectors);
    covs[i](3, 3) = 0.0;

    normals[i] << eigenvectors.col(0), 0.0;
    if (points[i].dot(normals[i]) > 0.0) {
      normals[i] = -normals[i];
    }
  }
}

std::vector<Eigen::Matrix4d> CloudCovarianceEstimation::estimate(const std::vector<Eigen::Vector4d>& points, const std::vector<int>& neighbors, const int k_neighbors) const {
  if (points.empty()) {
    return std::vector<Eigen::Matrix4d>();
  }

  const int k_correspondences = neighbors.size() / points.size();
  assert(k_correspondences * points.size() == neighbors.size());
  assert(k_neighbors <= k_correspondences);

  // Precompute pt * pt.transpose()
  std::vector<Eigen::Matrix4d> pt_cross(points.size());
  for (int i = 0; i < points.size(); i++) {
    pt_cross[i] = points[i] * points[i].transpose();
  }

  // Calculate covariances
  std::vector<Eigen::Matrix4d> covs(points.size());
  for (int i = 0; i < points.size(); i++) {
    Eigen::Vector4d sum_points = Eigen::Vector4d::Zero();
    Eigen::Matrix4d sum_cross = Eigen::Matrix4d::Zero();

    const int begin = k_correspondences * i;
    for (int j = 0; j < k_neighbors; j++) {
      const int index = neighbors[begin + j];
      sum_points += points[index];
      sum_cross += pt_cross[index];
    }

    const Eigen::Vector4d mean = sum_points / k_neighbors;
    const Eigen::Matrix4d cov = (sum_cross - mean * sum_points.transpose()) / (k_neighbors - 1);
    covs[i] = regularize(cov);
    covs[i](3, 3) = 0.0;
  }

  return covs;
}

std::vector<Eigen::Matrix4d> CloudCovarianceEstimation::estimate(const std::vector<Eigen::Vector4d>& points, const std::vector<int>& neighbors) const {
  if (points.empty()) {
    return std::vector<Eigen::Matrix4d>();
  }

  const int k = neighbors.size() / points.size();
  if (k * points.size() != neighbors.size()) {
    spdlog::critical("k * points.size() != neighbors.size()");
    abort();
  }

  return estimate(points, neighbors, k);
}

Eigen::Matrix4d CloudCovarianceEstimation::regularize(const Eigen::Matrix4d& cov, Eigen::Vector3d* eigenvalues, Eigen::Matrix3d* eigenvectors) const {
  switch (regularization_method) {
    default:
    case RegularizationMethod::NONE:
      return cov;

    case RegularizationMethod::PLANE: {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig;
      eig.computeDirect(cov.block<3, 3>(0, 0));

      if (eigenvalues) {
        *eigenvalues = eig.eigenvalues();
      }
      if (eigenvectors) {
        *eigenvectors = eig.eigenvectors();
      }

      Eigen::Vector3d values(plane_eps, 1.0, 1.0);
      Eigen::Matrix4d c = Eigen::Matrix4d::Zero();
      c.block<3, 3>(0, 0) = eig.eigenvectors() * values.asDiagonal() * eig.eigenvectors().transpose();
      return c;
    }

    case RegularizationMethod::NORMALIZED_MIN_EIG: {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig;
      eig.computeDirect(cov.block<3, 3>(0, 0));

      if (eigenvalues) {
        *eigenvalues = eig.eigenvalues();
      }
      if (eigenvectors) {
        *eigenvectors = eig.eigenvectors();
      }

      Eigen::Vector3d values = eig.eigenvalues() / eig.eigenvalues()[2];
      values = values.array().max(plane_eps);

      Eigen::Matrix4d c = Eigen::Matrix4d::Zero();
      c.block<3, 3>(0, 0) = eig.eigenvectors() * values.asDiagonal() * eig.eigenvectors().transpose();
      return c;
    }

    case RegularizationMethod::FROBENIUS: {
      const double lambda = plane_eps;
      Eigen::Matrix3d C = cov.block<3, 3>(0, 0) + lambda * Eigen::Matrix3d::Identity();
      Eigen::Matrix3d C_inv = C.inverse();
      Eigen::Matrix4d C_ = Eigen::Matrix4d::Zero();
      C_.block<3, 3>(0, 0) = (C_inv / C_inv.norm()).inverse();
      return C_;
    }
  }
}

}  // namespace glim