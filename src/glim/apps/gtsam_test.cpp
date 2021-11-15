#include <iostream>

#include <glim/backend/global_mapping.hpp>
#include <glim/backend/async_global_mapping.hpp>
#include <glim/viewer/interactive_viewer.hpp>

int main(int argc, char** argv) {
  std::shared_ptr<glim::InteractiveViewer> viewer(new glim::InteractiveViewer);
  std::shared_ptr<glim::GlobalMapping> global_mapping(new glim::GlobalMapping);
  global_mapping->load("/tmp/dump");
  std::shared_ptr<glim::AsyncGlobalMapping> async_global_mapping(new glim::AsyncGlobalMapping(global_mapping));

  viewer->wait();

  return 0;
}