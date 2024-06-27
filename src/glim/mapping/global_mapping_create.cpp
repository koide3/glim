#include <glim/mapping/global_mapping.hpp>

extern "C" glim::GlobalMappingBase* create_global_mapping_module() {
  glim::GlobalMappingParams params;
  return new glim::GlobalMapping(params);
}
