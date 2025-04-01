#include <glim/mapping/async_global_mapping.hpp>

#include <spdlog/spdlog.h>

#include <glim/util/logging.hpp>
#include <glim/mapping/callbacks.hpp>

namespace glim {

AsyncGlobalMapping::AsyncGlobalMapping(const std::shared_ptr<glim::GlobalMappingBase>& global_mapping, const int optimization_interval)
: global_mapping(global_mapping),
  optimization_interval(optimization_interval),
  logger(create_module_logger("global")) {
  request_to_optimize = false;
  request_to_recover = false;
  request_to_find_overlapping_submaps.store(-1.0);

  GlobalMappingCallbacks::request_to_optimize.add([this] { request_to_optimize = true; });
  GlobalMappingCallbacks::request_to_recover.add([this] { request_to_recover = true; });
  GlobalMappingCallbacks::request_to_find_overlapping_submaps.add([this](double min_overlap) { request_to_find_overlapping_submaps.store(min_overlap); });

  kill_switch = false;
  end_of_sequence = false;
  thread = std::thread([this] { run(); });
}

AsyncGlobalMapping::~AsyncGlobalMapping() {
  kill_switch = true;
  join();
}

#ifdef GLIM_USE_OPENCV
void AsyncGlobalMapping::insert_image(const double stamp, const cv::Mat& image) {
  input_image_queue.push_back(std::make_pair(stamp, image));
}
#endif

void AsyncGlobalMapping::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Eigen::Matrix<double, 7, 1> imu_data;
  imu_data << stamp, linear_acc, angular_vel;
  input_imu_queue.push_back(imu_data);
}

void AsyncGlobalMapping::insert_submap(const SubMap::Ptr& submap) {
  input_submap_queue.push_back(submap);
}

void AsyncGlobalMapping::join() {
  end_of_sequence = true;
  if (thread.joinable()) {
    thread.join();
  }

  // for (int i = 0; i < 64; i++) {
  //   global_mapping->optimize();
  // }
}

int AsyncGlobalMapping::workload() const {
  return input_submap_queue.size();
}

void AsyncGlobalMapping::save(const std::string& path) {
  logger->info("saving to {}...", path);
  std::lock_guard<std::mutex> lock(global_mapping_mutex);
  global_mapping->save(path);
  logger->info("saved");
}

std::vector<Eigen::Vector4d> AsyncGlobalMapping::export_points() {
  std::lock_guard<std::mutex> lock(global_mapping_mutex);
  logger->info("exporting points");
  auto points = global_mapping->export_points();
  return points;
}

void AsyncGlobalMapping::run() {
  auto last_optimization_time = std::chrono::high_resolution_clock::now();

  while (!kill_switch) {
#ifdef GLIM_USE_OPENCV
    auto images = input_image_queue.get_all_and_clear();
#endif
    auto imu_frames = input_imu_queue.get_all_and_clear();
    auto submaps = input_submap_queue.get_all_and_clear();

    if (
#ifdef GLIM_USE_OPENCV
      images.empty() &&
#endif
      imu_frames.empty() && submaps.empty()) {
      if (end_of_sequence) {
        break;
      }

      const double min_overlap = request_to_find_overlapping_submaps.exchange(-1.0);
      if (min_overlap > 0.0) {
        std::lock_guard<std::mutex> lock(global_mapping_mutex);
        global_mapping->find_overlapping_submaps(min_overlap);
      }

      if (request_to_optimize || std::chrono::high_resolution_clock::now() - last_optimization_time > std::chrono::seconds(optimization_interval)) {
        std::lock_guard<std::mutex> lock(global_mapping_mutex);
        request_to_optimize = false;
        global_mapping->optimize();
        last_optimization_time = std::chrono::high_resolution_clock::now();
      }

      if (request_to_recover) {
        std::lock_guard<std::mutex> lock(global_mapping_mutex);
        request_to_recover = false;
        global_mapping->recover_graph();
        last_optimization_time = std::chrono::high_resolution_clock::now();
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    std::lock_guard<std::mutex> lock(global_mapping_mutex);
    for (const auto& imu : imu_frames) {
      const double stamp = imu[0];
      const Eigen::Vector3d linear_acc = imu.block<3, 1>(1, 0);
      const Eigen::Vector3d angular_vel = imu.block<3, 1>(4, 0);
      global_mapping->insert_imu(stamp, linear_acc, angular_vel);
    }

#ifdef GLIM_USE_OPENCV
    for (const auto& image : images) {
      global_mapping->insert_image(image.first, image.second);
    }
#endif

    for (const auto& submap : submaps) {
      global_mapping->insert_submap(submap);
    }

    last_optimization_time = std::chrono::high_resolution_clock::now();
  }
}

}  // namespace glim