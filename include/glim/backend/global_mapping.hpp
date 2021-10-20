#pragma once

#include <memory>
#include <boost/shared_ptr.hpp>
#include <glim/backend/global_mapping_base.hpp>

namespace gtsam {
class Values;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace gtsam_ext {
class ISAM2Ext;
class StreamTempBufferRoundRobin;
}  // namespace gtsam_ext

namespace glim {

class IMUIntegration;

class GlobalMapping : public GlobalMappingBase {
public:
  GlobalMapping();
  virtual ~GlobalMapping();

  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;
  virtual void insert_submap(const SubMap::Ptr& submap) override;

  virtual void optimize() override;

private:
  boost::shared_ptr<gtsam::NonlinearFactorGraph> create_matching_cost_factors(int current) const;

  void update_submaps();

private:
  double max_implicit_loop_distance;
  double min_implicit_loop_overlap;

  std::unique_ptr<IMUIntegration> imu_integration;
  std::unique_ptr<gtsam_ext::StreamTempBufferRoundRobin> stream_buffer_roundrobin;

  std::vector<SubMap::Ptr> submaps;

  std::unique_ptr<gtsam::Values> new_values;
  std::unique_ptr<gtsam::NonlinearFactorGraph> new_factors;

  std::unique_ptr<gtsam_ext::ISAM2Ext> isam2;
};
}  // namespace glim