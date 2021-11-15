#pragma once

#include <memory>
#include <thread>

namespace glim {

class InteractiveViewer {
public:
  InteractiveViewer();
  ~InteractiveViewer();

  bool ok() const;
  void wait();
  void stop();

private:
  class Impl;
  std::unique_ptr<Impl> impl;
};

}