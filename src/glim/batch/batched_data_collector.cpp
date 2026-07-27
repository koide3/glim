#include <glim/batch/batched_data_collector.hpp>

#include <glim/common/callbacks.hpp>
#include <glim/util/callback_slot.hpp>

namespace glim {

BatchedDataCollector::BatchedDataCollector(int batch_size) : mutexes(batch_size), containers(batch_size) {
  for (int i = 0; i < batch_size; i++) {
    ScopedCallbackContext ctx(i);
    CommonCallbacks::on_extension_output.add([this](const std::string& module_name, const GenericDataContainer::ConstPtr& data) {
      std::lock_guard<std::mutex> lock(this->mutexes[CallbackContext::current()]);
      this->containers[CallbackContext::current()].emplace_back(module_name, data);
    });
  }
}

BatchedDataCollector::~BatchedDataCollector() {}

std::vector<std::vector<std::pair<std::string, GenericDataContainer::ConstPtr>>> BatchedDataCollector::get_and_clear() {
  std::vector<std::vector<std::pair<std::string, GenericDataContainer::ConstPtr>>> result(containers.size());
  for (size_t i = 0; i < containers.size(); i++) {
    std::lock_guard<std::mutex> lock(mutexes[i]);
    result[i] = std::move(containers[i]);
    containers[i].clear();
  }
  return result;
}

}  // namespace glim