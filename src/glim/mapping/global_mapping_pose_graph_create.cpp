#include <glim/mapping/global_mapping_pose_graph.hpp>

extern "C" glim::GlobalMappingBase* create_global_mapping_module() {
  glim::GlobalMappingPoseGraphParams params;
  return new glim::GlobalMappingPoseGraph(params);
}
