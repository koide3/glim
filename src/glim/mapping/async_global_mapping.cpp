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

  optimize_callback_id = GlobalMappingCallbacks::request_to_optimize.add([this] { request_to_optimize = true; });
  graph_factors_callback_id = GlobalMappingCallbacks::request_to_add_graph_factors.add([this](const gtsam::NonlinearFactorGraph& factors) {
    std::lock_guard<std::mutex> lock(graph_factors_request_mutex);
    graph_factors_request.add(factors);
  });
  merge_sessions_callback_id = GlobalMappingCallbacks::request_to_merge_sessions.add([this](const SessionMergeOptions& options) {
    std::lock_guard<std::mutex> lock(merge_request_mutex);
    merge_request = options;
  });
  recover_callback_id = GlobalMappingCallbacks::request_to_recover.add([this] { request_to_recover = true; });
  find_overlaps_callback_id =
    GlobalMappingCallbacks::request_to_find_overlapping_submaps.add([this](double min_overlap) { request_to_find_overlapping_submaps.store(min_overlap); });

  kill_switch = false;
  end_of_sequence = false;
  thread = std::thread([this] { run(); });
}

AsyncGlobalMapping::~AsyncGlobalMapping() {
  GlobalMappingCallbacks::request_to_optimize.remove(optimize_callback_id);
  GlobalMappingCallbacks::request_to_add_graph_factors.remove(graph_factors_callback_id);
  GlobalMappingCallbacks::request_to_merge_sessions.remove(merge_sessions_callback_id);
  GlobalMappingCallbacks::request_to_recover.remove(recover_callback_id);
  GlobalMappingCallbacks::request_to_find_overlapping_submaps.remove(find_overlaps_callback_id);

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

gtsam_points::PointCloud::Ptr AsyncGlobalMapping::export_points() {
  std::lock_guard<std::mutex> lock(global_mapping_mutex);
  logger->info("exporting points");
  logger->info("global_mapping: {}", global_mapping.get() == nullptr);
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

      std::optional<SessionMergeOptions> requested_merge;
      {
        std::lock_guard<std::mutex> lock(merge_request_mutex);
        requested_merge.swap(merge_request);
      }
      if (requested_merge) {
        std::lock_guard<std::mutex> lock(global_mapping_mutex);
        global_mapping->merge_sessions(*requested_merge);
        last_optimization_time = std::chrono::high_resolution_clock::now();
      }

      gtsam::NonlinearFactorGraph requested_graph_factors;
      {
        std::lock_guard<std::mutex> lock(graph_factors_request_mutex);
        requested_graph_factors = std::move(graph_factors_request);
        graph_factors_request = gtsam::NonlinearFactorGraph();
      }
      if (!requested_graph_factors.empty()) {
        std::lock_guard<std::mutex> lock(global_mapping_mutex);
        global_mapping->add_graph_factors(requested_graph_factors);
        last_optimization_time = std::chrono::high_resolution_clock::now();
      }

      if (request_to_optimize || std::chrono::high_resolution_clock::now() - last_optimization_time > std::chrono::seconds(optimization_interval)) {
        std::lock_guard<std::mutex> lock(global_mapping_mutex);
        request_to_optimize = false;
        if (global_mapping->graph_edit_state() == GraphEditState::IDLE) {
          global_mapping->optimize();
        }
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
