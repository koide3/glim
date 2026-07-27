#pragma once

#include <vector>
#include <string>
#include <memory>
#include <cstdint>
#include <variant>
#include <Eigen/Core>

namespace glim {

using DataAttribute = std::variant<
  // primitive types
  bool,
  std::int64_t,
  std::uint64_t,
  double,
  std::string,
  // vectors
  std::vector<int>,
  std::vector<float>,
  std::vector<double>,
  std::vector<std::string>,
  // Eigen types
  Eigen::Vector2i,
  Eigen::Vector3i,
  Eigen::Vector4i,
  Eigen::VectorXi,
  Eigen::Matrix2i,
  Eigen::Matrix3i,
  Eigen::Matrix4i,
  Eigen::MatrixXi,
  std::vector<Eigen::MatrixXi>,
  Eigen::Vector2f,
  Eigen::Vector3f,
  Eigen::Vector4f,
  Eigen::VectorXf,
  Eigen::Matrix2f,
  Eigen::Matrix3f,
  Eigen::Matrix4f,
  Eigen::MatrixXf,
  std::vector<Eigen::MatrixXf>,
  Eigen::Vector2d,
  Eigen::Vector3d,
  Eigen::Vector4d,
  Eigen::VectorXd,
  Eigen::Matrix2d,
  Eigen::Matrix3d,
  Eigen::Matrix4d,
  Eigen::MatrixXd,
  std::vector<Eigen::MatrixXd>
  //
  >;

struct GenericDataContainer {
public:
  using Ptr = std::shared_ptr<GenericDataContainer>;
  using ConstPtr = std::shared_ptr<const GenericDataContainer>;

  template <typename T>
  void emplace_back(const std::string& key, T&& value) {
    attributes.emplace_back(key, std::forward<T>(value));
  }

public:
  double stamp;
  std::vector<std::pair<std::string, DataAttribute>> attributes;
};

}  // namespace glim