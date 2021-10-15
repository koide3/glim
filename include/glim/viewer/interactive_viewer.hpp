#pragma once

#include <memory>
#include <thread>

namespace glim {

class InteractiveViewer {
public:
  InteractiveViewer();
  ~InteractiveViewer();

private:
  class Impl;
  std::unique_ptr<Impl> impl;
};
}