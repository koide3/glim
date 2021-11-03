#include <fstream>
#include <iostream>
#include <boost/format.hpp>
#include <portable-file-dialogs.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam_ext/types/voxelized_frame.hpp>
#include <gtsam_ext/types/voxelized_frame_cpu.hpp>
#include <gtsam_ext/types/voxelized_frame_gpu.hpp>
#include <gtsam_ext/factors/integrated_gicp_factor.hpp>
#include <gtsam_ext/factors/integrated_vgicp_factor.hpp>
#include <gtsam_ext/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_ext/optimizers/isam2_ext.hpp>
#include <gtsam_ext/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_ext/cuda/stream_temp_buffer_roundrobin.hpp>

#include <glk/io/ply_io.hpp>
#include <glk/thin_lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/recent_files.hpp>
#include <guik/progress_modal.hpp>
#include <guik/viewer/light_viewer.hpp>

#include <glim/util/console_colors.hpp>
#include <glim/backend/sub_map.hpp>

namespace glim {

using gtsam::symbol_shorthand::X;

struct GlobalMap {
public:
  static std::shared_ptr<GlobalMap> load(const std::string& path) {
    std::ifstream ifs(path + "/graph.txt");
    if (!ifs) {
      std::cerr << console::bold_red << "error: failed to open " << path + "/graph.txt" << console::reset << std::endl;
      return nullptr;
    }

    std::shared_ptr<GlobalMap> global_map(new GlobalMap);

    std::string token;
    int num_submaps, num_all_frames, num_factors;
    ifs >> token >> num_submaps;
    ifs >> token >> num_all_frames;
    ifs >> token >> num_factors;

    for (int i = 0; i < num_factors; i++) {
      std::pair<int, int> factor;
      ifs >> token >> factor.first >> factor.second;

      if (token.find("between") != std::string::npos) {
        global_map->between_factors.push_back(factor);
      }
      if (token.find("matching_cost") != std::string::npos) {
        global_map->matching_cost_factors.push_back(factor);
      }
    }

    for (int i = 0; i < num_submaps; i++) {
      const auto submap = SubMap::load((boost::format("%s/%06d") % path % i).str());
      if (submap == nullptr) {
        std::cerr << console::bold_red << "error: failed to load " << boost::format("%s/%06d") % path % i << console::reset << std::endl;
        return nullptr;
      }
      global_map->submaps.push_back(submap);
      global_map->voxelized_submaps.push_back(std::make_shared<gtsam_ext::VoxelizedFrameGPU>(1.0, *submap->frame));
    }

    return global_map;
  }

  void recreate_factors(guik::ProgressInterface& progress) {
    progress.set_title("Recreating factors");
    progress.set_maximum(submaps.size());

    matching_cost_factors.clear();
    for (int i = 0; i < submaps.size(); i++) {
      for (int j = i + 1; j < submaps.size(); j++) {
        const Eigen::Isometry3d delta = submaps[i]->T_world_origin.inverse() * submaps[j]->T_world_origin;
        const double overlap = voxelized_submaps[j]->overlap_auto(voxelized_submaps[i], delta);

        if (overlap > 0.05) {
          matching_cost_factors.push_back(std::make_pair(i, j));
        }
      }
    }
  }

  void optimize(guik::ProgressInterface& progress) {
    progress.set_title("Optimization");

    gtsam::Values values;
    for (int i = 0; i < submaps.size(); i++) {
      values.insert(X(i), gtsam::Pose3(submaps[i]->T_world_origin.matrix()));
    }

    progress.set_text("Building graph");
    progress.set_current(0);
    progress.set_maximum(matching_cost_factors.size());

    gtsam::NonlinearFactorGraph graph;
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), values.at<gtsam::Pose3>(X(0)), gtsam::noiseModel::Isotropic::Precision(6, 1e6));

    std::unique_ptr<gtsam_ext::StreamTempBufferRoundRobin> stream_buffer_roundrobin(new gtsam_ext::StreamTempBufferRoundRobin(64));
    for (const auto& factor : matching_cost_factors) {
      progress.increment();

      const auto stream_buffer = stream_buffer_roundrobin->get_stream_buffer();
      const auto& stream = stream_buffer.first;
      const auto& buffer = stream_buffer.second;
      graph
        .emplace_shared<gtsam_ext::IntegratedVGICPFactorGPU>(X(factor.first), X(factor.second), voxelized_submaps[factor.first], voxelized_submaps[factor.second], stream, buffer);
    }

    progress.set_text("Optimizing");
    progress.set_current(0);
    progress.set_maximum(10);

    gtsam_ext::LevenbergMarquardtExtParams lm_params;
    lm_params.callback = [&](const gtsam_ext::LevenbergMarquardtOptimizationStatus& status, const gtsam::Values& values) {
      progress.increment();
      std::cout << status.to_string() << std::endl;
    };
    gtsam_ext::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);
    values = optimizer.optimize();

    for (int i = 0; i < submaps.size(); i++) {
      submaps[i]->T_world_origin = Eigen::Isometry3d(values.at<gtsam::Pose3>(X(i)).matrix());
    }
  }

  void save_pointcloud(const std::string& path) {
    long long num_all_points = 0;
    for (const auto& submap : submaps) {
      num_all_points += submap->frame->size();
    }

    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> all_points;
    all_points.reserve(num_all_points);
    for (const auto& submap : submaps) {
      std::transform(submap->frame->points, submap->frame->points + submap->frame->size(), std::back_inserter(all_points), [&](const Eigen::Vector4d& pt) {
        return submap->T_world_origin * pt;
      });
    }

    glk::save_ply_binary(path, all_points.data(), all_points.size());
  }

public:
  std::vector<SubMap::Ptr> submaps;
  std::vector<gtsam_ext::VoxelizedFrame::Ptr> voxelized_submaps;

  std::vector<std::pair<int, int>> between_factors;
  std::vector<std::pair<int, int>> matching_cost_factors;
};

class GlobalMapViewer {
public:
  GlobalMapViewer(const std::shared_ptr<GlobalMap>& global_map) : global_map(global_map) {
    auto viewer = guik::LightViewer::instance();
    update_viewer();

    progress.reset(new guik::ProgressModal("progress"));

    viewer->register_ui_callback("ui", [this] { ui_callback(); });
  }

  void ui_callback() {
    if (ImGui::Button("Recreate factors")) {
      progress->open<bool>("recreate_factors", [this](guik::ProgressInterface& progress) {
        global_map->recreate_factors(progress);
        return true;
      });
    }

    if (ImGui::Button("Optimize")) {
      progress->open<bool>("optimize", [this](guik::ProgressInterface& progress) {
        global_map->optimize(progress);
        return true;
      });
    }

    if (progress->run<bool>("recreate_factors") || progress->run<bool>("optimize")) {
      update_viewer();
    }
  }

  void update_viewer() {
    auto viewer = guik::LightViewer::instance();
    for (const auto& submap : global_map->submaps) {
      auto drawable = viewer->find_drawable("submap_" + std::to_string(submap->id));
      if (drawable.first) {
        drawable.first->add("model_matrix", submap->T_world_origin.cast<float>().matrix());
      } else {
        auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(submap->frame->points, submap->frame->size());
        viewer->update_drawable("submap_" + std::to_string(submap->id), cloud_buffer, guik::Rainbow(submap->T_world_origin.cast<float>()));
      }

      viewer->update_drawable(
        "submap_coord_" + std::to_string(submap->id),
        glk::Primitives::coordinate_system(),
        guik::VertexColor(submap->T_world_origin.cast<float>() * Eigen::UniformScaling<float>(5.0f)));
    }

    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> factor_lines;
    for (const auto& factor : global_map->matching_cost_factors) {
      factor_lines.push_back(global_map->submaps[factor.first]->T_world_origin.translation().cast<float>());
      factor_lines.push_back(global_map->submaps[factor.second]->T_world_origin.translation().cast<float>());
    }
    viewer->update_drawable("factors", std::make_shared<glk::ThinLines>(factor_lines), guik::FlatGreen());
  }

private:
  std::unique_ptr<guik::ProgressModal> progress;

  std::shared_ptr<GlobalMap> global_map;
};

}  // namespace glim

int main(int argc, char** argv) {
  guik::RecentFiles recent_files("input_dump_directory");
  const std::string input_path = pfd::select_folder("Select dump directory", recent_files.most_recent()).result();
  if (input_path.empty()) {
    return 1;
  }
  recent_files.push(input_path);

  auto global_map = glim::GlobalMap::load(input_path);
  if (!global_map) {
    return 1;
  }

  glim::GlobalMapViewer viewer(global_map);
  guik::LightViewer::instance()->spin();

  /*
  global_map->optimize();

  guik::RecentFiles recent_files2("output_dump_directory");
  const std::string output_path = pfd::save_file("Select dump dst path", recent_files2.most_recent()).result();
  if (output_path.size()) {
    recent_files2.push(output_path);
    global_map->save_pointcloud(output_path);
  }
  */

  return 0;
}