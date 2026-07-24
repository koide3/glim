#pragma once

#include <mutex>
#include <atomic>
#include <thread>
#include <memory>
#include <vector>
#include <unordered_map>
#include <optional>
#include <boost/weak_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam_points/util/gtsam_migration.hpp>
#include <glim/util/extension_module.hpp>

namespace spdlog {
class logger;
}

namespace gtsam {
class NonlinearFactor;
}

namespace glim {

class TrajectoryManager;
struct EstimationFrame;

class StandardSubViewer : public ExtensionModule {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StandardSubViewer();
  virtual ~StandardSubViewer();

  virtual bool ok() const override;

  void invoke(const std::function<void()>& task);

private:
  Eigen::Isometry3f resolve_pose(const std::shared_ptr<const EstimationFrame>& frame);

  void set_callbacks();

  bool drawable_filter(const std::string& name);
  void drawable_selection();

private:
  int context_id;
  std::string sub_viewer_name;
  std::unique_ptr<TrajectoryManager> trajectory;

  // Logging
  std::shared_ptr<spdlog::logger> logger;
};
}  // namespace glim