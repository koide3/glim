#pragma once

#include <memory>
#include <thread>

namespace glim {

class StandardViewer {
public:
  StandardViewer();
  ~StandardViewer();

private:
  class Impl;
  std::unique_ptr<Impl> impl;
};
}