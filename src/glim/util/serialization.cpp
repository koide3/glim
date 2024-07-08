#include <glim/util/serialization.hpp>

#include <spdlog/spdlog.h>
#include <gtsam/base/serialization.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace glim {

void serializeToBinaryFile(const gtsam::NonlinearFactorGraph& graph, const std::string& path, bool only_serializable) {
  try {
    gtsam::serializeToBinaryFile(graph, path);
    return;
  } catch (boost::archive::archive_exception e) {
    spdlog::warn("failed to serialize factor graph!!");
    spdlog::warn(e.what());

    if (!only_serializable) {
      throw e;
    }
  }

  spdlog::warn("retyring to serialize factor graph with only serializable factor types");

  gtsam::NonlinearFactorGraph ser;
  for (const auto& factor : graph) {
    try {
      gtsam::serializeBinary(factor);
      ser.add(factor);
    } catch (boost::archive::archive_exception e) {
      factor->print();
    }
  }

  gtsam::serializeToBinaryFile(ser, path);
}

void serializeToBinaryFile(const gtsam::Values& values, const std::string& path, bool only_serializable) {
  try {
    gtsam::serializeToBinaryFile(values, path);
    return;
  } catch (boost::archive::archive_exception e) {
    spdlog::warn("failed to serialize values!!");
    spdlog::warn(e.what());

    if (!only_serializable) {
      throw e;
    }
  }

  spdlog::warn("retyring to serialize values with only serializable value types");

  gtsam::Values ser;
  for (const auto& value : values) {
    try {
      gtsam::serializeBinary(value.value);
      ser.insert(value.key, value.value);
    } catch (boost::archive::archive_exception e) {
      std::cout << "key=" << value.key << std::endl;
      value.value.print();
    }
  }

  gtsam::serializeToBinaryFile(ser, path);
}

}  // namespace glim
