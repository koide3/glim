#include <glim/backend/sub_map.hpp>

#include <spdlog/spdlog.h>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <gtsam_ext/types/point_cloud_cpu.hpp>

#include <glim/util/console_colors.hpp>

namespace glim {

void SubMap::drop_frame_points() {
  for (auto& frame : frames) {
    frame = frame->clone_wo_points();
  }

  for (auto& frame : odom_frames) {
    frame = frame->clone_wo_points();
  }
}

void SubMap::save(const std::string& path) const {
  boost::filesystem::create_directories(path);
  std::ofstream ofs(path + "/data.txt");
  ofs << "id: " << id << std::endl;
  ofs << "T_world_origin: " << std::endl << T_world_origin.matrix() << std::endl;
  ofs << "T_origin_endpoint_L: " << std::endl << T_origin_endpoint_L.matrix() << std::endl;
  ofs << "T_origin_endpoint_R: " << std::endl << T_origin_endpoint_R.matrix() << std::endl;

  if (!frames.empty()) {
    ofs << "T_lidar_imu: " << std::endl << frames.back()->T_lidar_imu.matrix() << std::endl;
    ofs << "imu_bias: " << frames.back()->imu_bias.transpose() << std::endl;
    ofs << "frame_id: " << static_cast<int>(frames.back()->frame_id) << std::endl;
  }

  ofs << "num_frames: " << frames.size() << std::endl;

  for (int i = 0; i < frames.size(); i++) {
    ofs << "frame_" << i << std::endl;
    ofs << "id: " << frames[i]->id << std::endl;
    ofs << "stamp: " << boost::format("%.9f") % frames[i]->stamp << std::endl;
    ofs << "T_odom_lidar: " << std::endl << odom_frames[i]->T_world_lidar.matrix() << std::endl;
    ofs << "T_world_lidar: " << std::endl << frames[i]->T_world_lidar.matrix() << std::endl;
    ofs << "v_world_imu: " << frames[i]->v_world_imu.transpose() << std::endl;
  }

  std::ofstream ofs_imu_rate(path + "/imu_rate.txt");
  for (const auto& frame : frames) {
    if (!frame->imu_rate_trajectory.size()) {
      continue;
    }

    for (int i = 0; i < frame->imu_rate_trajectory.cols(); i++) {
      const auto& data = frame->imu_rate_trajectory.col(i);
      ofs_imu_rate << boost::format("%.9f %.6f %.6f %.6f %.6f %.6f %.6f %.6f") % data[0] % data[1] % data[2] % data[3] % data[4] % data[5] % data[6] % data[7] << std::endl;
    }
  }

  frame->save_compact(path);
}

namespace {
template <int ROWS, int COLS>
Eigen::Matrix<double, ROWS, COLS> read_matrix(std::ifstream& ifs) {
  Eigen::Matrix<double, ROWS, COLS> m;
  for (int i = 0; i < ROWS; i++) {
    for (int j = 0; j < COLS; j++) {
      ifs >> m(i, j);
    }
  }
  return m;
}
}  // namespace

SubMap::Ptr SubMap::load(const std::string& path) {
  std::ifstream ifs(path + "/data.txt");
  if (!ifs) {
    spdlog::error("failed to open {}/data.txt", path);
    return nullptr;
  }

  SubMap::Ptr submap(new SubMap);

  std::string token;
  ifs >> token >> submap->id;

  ifs >> token;
  submap->T_world_origin.matrix() = read_matrix<4, 4>(ifs);
  ifs >> token;
  submap->T_origin_endpoint_L.matrix() = read_matrix<4, 4>(ifs);
  ifs >> token;
  submap->T_origin_endpoint_R.matrix() = read_matrix<4, 4>(ifs);

  ifs >> token;
  Eigen::Isometry3d T_lidar_imu(read_matrix<4, 4>(ifs));
  ifs >> token;
  Eigen::Matrix<double, 6, 1> imu_bias = read_matrix<6, 1>(ifs);

  int frame_id;
  ifs >> token >> frame_id;

  int num_frames;
  ifs >> token >> num_frames;

  for (int i = 0; i < num_frames; i++) {
    int id;
    double stamp;
    ifs >> token >> token >> id;
    ifs >> token >> stamp;

    ifs >> token;
    Eigen::Isometry3d T_odom_lidar(read_matrix<4, 4>(ifs));

    ifs >> token;
    Eigen::Isometry3d T_world_lidar(read_matrix<4, 4>(ifs));

    ifs >> token;
    Eigen::Vector3d v_world_imu = read_matrix<3, 1>(ifs);

    EstimationFrame::Ptr frame(new EstimationFrame);
    frame->id = id;
    frame->stamp = stamp;
    frame->T_lidar_imu = T_lidar_imu;
    frame->T_world_lidar = T_world_lidar;
    frame->T_world_imu = T_world_lidar * T_lidar_imu;

    frame->v_world_imu = v_world_imu;
    frame->imu_bias = imu_bias;
    frame->frame_id = static_cast<FrameID>(frame_id);

    EstimationFrame::Ptr odom_frame(new EstimationFrame);
    *odom_frame = *frame;
    odom_frame->T_world_lidar = T_odom_lidar;
    odom_frame->T_world_imu = T_odom_lidar * T_lidar_imu;

    submap->frames.push_back(frame);
    submap->odom_frames.push_back(odom_frame);
  }

  submap->frame = gtsam_ext::PointCloudCPU::load(path);

  return submap;
}

}  // namespace glim
