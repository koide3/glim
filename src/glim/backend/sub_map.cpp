#include <glim/backend/sub_map.hpp>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

namespace glim {

void SubMap::drop_odom_frames() {
  for (auto& frame : frames) {
    frame = frame->clone_wo_points();
  }

  for (auto& frame : odom_frames) {
    frame = frame->clone_wo_points();
  }
}

void SubMap::save(const std::string& path) {
  boost::filesystem::create_directories(path);
  std::ofstream ofs(path + "/data.txt");
  ofs << "id: " << id << std::endl;
  ofs << "voxel_resolution: " << frame->voxel_resolution() << std::endl;
  ofs << "T_world_origin: " << std::endl << T_world_origin.matrix() << std::endl;
  ofs << "T_origin_endpoint_L: " << std::endl << T_origin_endpoint_L.matrix() << std::endl;
  ofs << "T_origin_endpoint_R: " << std::endl << T_origin_endpoint_R.matrix() << std::endl;

  if (!frames.empty()) {
    ofs << "T_lidar_imu: " << std::endl << frames.back()->T_lidar_imu.matrix() << std::endl;
    ofs << "imu_bias: " << frames.back()->imu_bias.transpose() << std::endl;
    ofs << "frame_id: " << frames.back()->frame_id << std::endl;
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

  frame->save_compact(path);
}

}  // namespace glim
