#include <glim/mapping/sub_mapping.hpp>

extern "C" glim::SubMappingBase* create_sub_mapping_module() {
  glim::SubMappingParams params;
  return new glim::SubMapping(params);
}
