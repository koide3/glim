#include <memory>
#include <iostream>

#include <guik/viewer/light_viewer.hpp>

#include <glim/backend/global_mapping.hpp>
#include <glim/backend/async_global_mapping.hpp>
#include <glim/viewer/interactive_viewer.hpp>

namespace glim {

class OfflineViewer : public InteractiveViewer {
public:
  OfflineViewer() {}
  ~OfflineViewer() {}

private:
  virtual void setup_ui() override {
    auto viewer = guik::LightViewer::instance();
    viewer->register_ui_callback("main_menu", [this] { main_menu(); });
  }

  void main_menu() {
    if (ImGui::BeginMainMenuBar()) {
      if (ImGui::BeginMenu("File")) {
        if (ImGui::MenuItem("Open")) {
        }

        if (ImGui::MenuItem("Quit")) {
          request_to_terminate = true;
        }

        ImGui::EndMenu();
      }

      ImGui::EndMainMenuBar();
    }
  }

private:
};

}  // namespace glim
