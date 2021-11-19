#include <glim/frontend/async_odometry_estimation.hpp>

namespace glim {

AsyncOdometryEstimation::AsyncOdometryEstimation(const std::shared_ptr<OdometryEstimationBase>& odometry_estimation) : odometry_estimation(odometry_estimation) {
  kill_switch = false;
  end_of_sequence = false;
  thread = std::thread([this] { run(); });
}

AsyncOdometryEstimation::~AsyncOdometryEstimation() {
  kill_switch = true;
  join();
}

void AsyncOdometryEstimation::insert_image(const double stamp, const cv::Mat& image) {
  input_image_queue.push_back(std::make_pair(stamp, image));
}

void AsyncOdometryEstimation::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Eigen::Matrix<double, 7, 1> imu_data;
  imu_data << stamp, linear_acc, angular_vel;
  input_imu_queue.push_back(imu_data);
}

void AsyncOdometryEstimation::insert_frame(const PreprocessedFrame::Ptr& frame) {
  input_frame_queue.push_back(frame);
}

void AsyncOdometryEstimation::join() {
  end_of_sequence = true;
  if(thread.joinable()) {
    thread.join();
  }
}

int AsyncOdometryEstimation::input_queue_size() const {
  return input_frame_queue.size();
}

int AsyncOdometryEstimation::output_queue_size() const {
  return output_estimation_results.size();
}

void AsyncOdometryEstimation::get_results(std::vector<EstimationFrame::ConstPtr>& estimation_results, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  estimation_results = output_estimation_results.get_all_and_clear();
  marginalized_frames = output_marginalized_frames.get_all_and_clear();
}

void AsyncOdometryEstimation::run() {
  double last_imu_time = 0.0;
  std::deque<std::pair<double, cv::Mat>> images;
  std::deque<PreprocessedFrame::Ptr> raw_frames;

  while (!kill_switch) {
    auto imu_frames = input_imu_queue.get_all_and_clear();
    auto new_images = input_image_queue.get_all_and_clear();
    auto new_raw_frames = input_frame_queue.get_all_and_clear();

    images.insert(images.end(), new_images.begin(), new_images.end());
    raw_frames.insert(raw_frames.end(), new_raw_frames.begin(), new_raw_frames.end());

    if(images.empty() && imu_frames.empty() && raw_frames.empty()) {
      if(end_of_sequence) {
        break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    for (const auto& imu : imu_frames) {
      const double stamp = imu[0];
      const Eigen::Vector3d linear_acc = imu.block<3, 1>(1, 0);
      const Eigen::Vector3d angular_vel = imu.block<3, 1>(4, 0);
      odometry_estimation->insert_imu(stamp, linear_acc, angular_vel);

      last_imu_time = stamp;
    }

    while (!images.empty() && images.front().first < last_imu_time) {
      const auto image = images.front();
      odometry_estimation->insert_image(image.first, image.second);
      images.pop_front();
    }

    while (!raw_frames.empty() && raw_frames.front()->scan_end_time < last_imu_time) {
      const auto& frame = raw_frames.front();
      std::vector<EstimationFrame::ConstPtr> marginalized;
      auto state = odometry_estimation->insert_frame(frame, marginalized);

      output_estimation_results.push_back(state);
      output_marginalized_frames.insert(marginalized);
      raw_frames.pop_front();
    }
  }

  auto marginalized = odometry_estimation->get_remaining_frames();
  output_marginalized_frames.insert(marginalized);
}
}