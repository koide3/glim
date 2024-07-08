#pragma once

#include <string>

namespace gtsam {
class Values;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace glim {

void serializeToBinaryFile(const gtsam::NonlinearFactorGraph& graph, const std::string& path, bool only_serializable = true);
void serializeToBinaryFile(const gtsam::Values& values, const std::string& path, bool only_serializable = true);

}  // namespace glim
