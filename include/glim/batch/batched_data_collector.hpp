#pragma once

#include <mutex>
#include <memory>
#include <glim/util/generic_data_container.hpp>

namespace glim {

/**
 * @brief BatchedDataCollector collects outputs from extension modules in a batched manner.
 */
class BatchedDataCollector {
public:
  BatchedDataCollector(int batch_size);
  ~BatchedDataCollector();

  std::vector<std::vector<std::pair<std::string, GenericDataContainer::ConstPtr>>> get_and_clear();

private:
  std::vector<std::mutex> mutexes;
  std::vector<std::vector<std::pair<std::string, GenericDataContainer::ConstPtr>>> containers;
};

}  // namespace glim