#pragma once

#include <glim/mapping/global_mapping.hpp>
#include <glim/mapping/async_global_mapping.hpp>
#include <glim/viewer/interactive_viewer.hpp>

namespace guik {
class ProgressModal;
class ProgressInterface;
}  // namespace guik

namespace glim {

class OfflineViewer : public InteractiveViewer {
public:
  OfflineViewer();
  virtual ~OfflineViewer() override;

private:
  virtual void setup_ui() override;

  void main_menu();

  std::shared_ptr<GlobalMapping> load_map(guik::ProgressInterface& progress, const std::string& path);
  bool save_map(guik::ProgressInterface& progress, const std::string& path);
  bool export_map(guik::ProgressInterface& progress, const std::string& path);

private:
  std::unique_ptr<guik::ProgressModal> progress_modal;

  std::unique_ptr<AsyncGlobalMapping> async_global_mapping;
};

}  // namespace glim
