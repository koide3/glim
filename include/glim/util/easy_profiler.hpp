#pragma once

#include <chrono>
#include <vector>
#include <iostream>

namespace glim {

class EasyProfiler {
public:
  EasyProfiler(const std::string& prof_label, bool enabled = true, bool debug = false, std::ostream& ost = std::cout)
  : enabled(enabled),
    debug(debug),
    prof_label(prof_label),
    ost(ost) {
    if (!enabled) {
      return;
    }

    labels.push_back("begin");
    times.push_back(std::chrono::high_resolution_clock::now());

    if (debug) {
      ost << "--- " << prof_label << " (debug) ---" << std::endl;
    }
  }

  ~EasyProfiler() {
    if (!enabled) {
      return;
    }

    labels.push_back("end");
    times.push_back(std::chrono::high_resolution_clock::now());

    ost << "--- " << prof_label << " ---" << '\n';

    int longest = 0;
    for (const auto& label : labels) {
      longest = std::max<int>(label.size(), longest);
    }

    for (int i = 1; i < labels.size(); i++) {
      std::vector<char> pad(longest - labels[i - 1].size(), ' ');
      std::string label = labels[i - 1] + std::string(pad.begin(), pad.end());

      ost << label << ":" << std::chrono::duration_cast<std::chrono::nanoseconds>(times[i] - times[i - 1]).count() / 1e6 << "[msec]" << '\n';
    }

    ost << "total:" << std::chrono::duration_cast<std::chrono::nanoseconds>(times.back() - times.front()).count() / 1e6 << "[msec]" << '\n';
    ost << std::flush;
  }

  void push(const std::string& label) {
    if (!enabled) {
      return;
    }

    labels.push_back(label);
    times.push_back(std::chrono::high_resolution_clock::now());

    if (debug) {
      ost << ">> " << label << " (" << std::chrono::duration_cast<std::chrono::nanoseconds>(times.back() - times.front()).count() / 1e6 << "[msec])" << std::endl;
    }
  }

private:
  const bool enabled;
  const bool debug;
  const std::string prof_label;

  std::vector<std::string> labels;
  std::vector<std::chrono::high_resolution_clock::time_point> times;

  std::ostream& ost;
};
}  // namespace glim