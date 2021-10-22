#pragma once

#include <memory>
#include <random>
#include <boost/shared_ptr.hpp>
#include <glim/backend/global_mapping_base.hpp>

namespace gtsam {
class Values;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace gtsam_ext {
class ISAM2Ext;
}  // namespace gtsam_ext

namespace glim {

class IMUIntegration;

class GlobalMappingCT : public GlobalMappingBase {
public:
  GlobalMappingCT();
  virtual ~GlobalMappingCT();

  virtual void insert_submap(const SubMap::Ptr& submap) override;

  virtual void optimize() override;

private:
  boost::shared_ptr<gtsam::NonlinearFactorGraph> create_consecutive_factors(int current) const;
  boost::shared_ptr<gtsam::NonlinearFactorGraph> create_matching_cost_factors(int current) const;

  void update_submaps();

private:
  double randomsampling_rate;
  double max_implicit_loop_distance;
  double min_implicit_loop_overlap;

  std::mt19937 mt;
  std::vector<SubMap::Ptr> submaps;
  std::vector<gtsam_ext::Frame::ConstPtr> subsampled_submaps;

  std::unique_ptr<gtsam::Values> new_values;
  std::unique_ptr<gtsam::NonlinearFactorGraph> new_factors;

  std::unique_ptr<gtsam_ext::ISAM2Ext> isam2;
};
}  // namespace glim