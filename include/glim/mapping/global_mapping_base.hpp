#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#ifdef GLIM_USE_OPENCV
#include <opencv2/core.hpp>
#endif

#include <glim/mapping/sub_map.hpp>
#include <gtsam_points/types/point_cloud.hpp>

namespace spdlog {
class logger;
}

namespace gtsam {
class NonlinearFactorGraph;
}

namespace glim {

enum class GraphEditState;
struct SessionMergeOptions;

/**
 * @brief Global mapping base class
 *
 */
class GlobalMappingBase {
public:
  GlobalMappingBase();
  virtual ~GlobalMappingBase() {}

#ifdef GLIM_USE_OPENCV
  /**
   * @brief Insert an image
   * @param stamp   Timestamp
   * @param image   Image
   */
  virtual void insert_image(const double stamp, const cv::Mat& image);
#endif

  /**
   * @brief Insert an IMU frame
   * @param stamp         Timestamp
   * @param linear_acc    Linear acceleration
   * @param angular_vel   Angular velocity
   */
  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);

  /**
   * @brief Insert a SubMap
   * @param submap  SubMap
   */
  virtual void insert_submap(const SubMap::Ptr& submap);

  /**
   * @brief Request to find new overlapping submaps
   */
  virtual void find_overlapping_submaps(double min_overlap);

  /**
   * @brief Add factors to the graph currently accepting edits
   * @param factors Factors to add
   */
  virtual void add_graph_factors(const gtsam::NonlinearFactorGraph& factors);

  /**
   * @brief Request to perform optimization
   */
  virtual void optimize();

  /**
   * @brief Get the current graph editing state
   */
  virtual GraphEditState graph_edit_state() const;

  /**
   * @brief Build and commit a candidate for the pending session merge
   * @param options Frozen session merge options
   */
  virtual void merge_sessions(const SessionMergeOptions& options);

  /**
   * @brief Request to detect and recover graph corruption
   */
  virtual void recover_graph();

  /**
   * @brief Save the mapping result
   * @param path  Save path
   */
  virtual void save(const std::string& path) {}

  /**
   * @brief Export all the submap points
   */
  virtual gtsam_points::PointCloud::Ptr export_points() { return nullptr; }

  /**
   * @brief Load a global mapping module from a shared library
   * @param so_name  Shared library name
   * @return         Loaded global mapping module
   */
  static std::shared_ptr<GlobalMappingBase> load_module(const std::string& so_name);

protected:
  std::shared_ptr<spdlog::logger> logger;
};
}  // namespace glim
