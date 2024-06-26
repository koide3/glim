#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glim {

/**
 * @brief Covariance regularization method
 */
enum class RegularizationMethod { NONE, PLANE, NORMALIZED_MIN_EIG, FROBENIUS };

/**
 * @brief Point covariance estimation
 */
class CloudCovarianceEstimation {
public:
  CloudCovarianceEstimation(const int num_threads = 1);
  ~CloudCovarianceEstimation();

  /**
   * @brief Estimate point normals and covariances
   * @param points    Input points
   * @param neighbors Neighbor indices (must be N * k, where N is the number of points)
   * @param normals   [output] Estimated normals
   * @param covs      [output] Estimated covariances
   */
  void estimate(const std::vector<Eigen::Vector4d>& points, const std::vector<int>& neighbors, std::vector<Eigen::Vector4d>& normals, std::vector<Eigen::Matrix4d>& covs) const;

  /**
   * @brief Estimate point normals and covariances
   * @param points      Input points
   * @param neighbors   Neighbor indices (must be N * m, where N is the number of points)
   * @param k_neighbors Number of neighbors used for estimation (must be <= m)
   * @param normals     [output] Estimated normals
   * @param covs        [output] Estimated covariances
   */
  void estimate(
    const std::vector<Eigen::Vector4d>& points,
    const std::vector<int>& neighbors,
    const int k_neighbors,
    std::vector<Eigen::Vector4d>& normals,
    std::vector<Eigen::Matrix4d>& covs) const;

  /// Estimate point covariances
  std::vector<Eigen::Matrix4d> estimate(const std::vector<Eigen::Vector4d>& points, const std::vector<int>& neighbors, const int k_neighbors) const;

  /// Estimate point covariances
  std::vector<Eigen::Matrix4d> estimate(const std::vector<Eigen::Vector4d>& points, const std::vector<int>& neighbors) const;

  /**
   * @brief Regularize a covariance matrix
   * @param cov          Input covariance matrix
   * @param eigenvalues  [output] Eigenvalues of the covariance matrix
   * @param eigenvectors [output] Eigenvectors of the covariance matrix
   * @return Regularized covariance matrix
   */
  Eigen::Matrix4d regularize(const Eigen::Matrix4d& cov, Eigen::Vector3d* eigenvalues = nullptr, Eigen::Matrix3d* eigenvectors = nullptr) const;

private:
  const RegularizationMethod regularization_method;
  const int num_threads;
};

}  // namespace glim
