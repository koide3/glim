#include <glim/mapping/sub_mapping_passthrough.hpp>

extern "C" glim::SubMappingBase* create_sub_mapping_module() {
  glim::SubMappingPassthroughParams params;
  return new glim::SubMappingPassthrough(params);
}
