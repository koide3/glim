#include <spdlog/spdlog.h>
#include <gtsam/config.h>
#include <gtsam_points/config.hpp>
#include <gtsam_points/util/parallelism.hpp>
#include <gtsam_points/cuda/cuda_device_prop.hpp>

#ifdef GTSAM_USE_SYSTEM_EIGEN
#define IS_DEFINED_GTSAM_USE_SYSTEM_EIGEN 1
#else
#define IS_DEFINED_GTSAM_USE_SYSTEM_EIGEN 0
#endif

#ifdef GTSAM_USE_SYSTEM_METIS
#define IS_DEFINED_GTSAM_USE_SYSTEM_METIS 1
#else
#define IS_DEFINED_GTSAM_USE_SYSTEM_METIS 0
#endif

#ifdef GTSAM_USE_TBB
#define IS_DEFINED_GTSAM_USE_TBB 1
#else
#define IS_DEFINED_GTSAM_USE_TBB 0
#endif

#ifdef GTSAM_ALLOCATOR_TBB
#define IS_DEFINED_GTSAM_ALLOCATOR_TBB 1
#else
#define IS_DEFINED_GTSAM_ALLOCATOR_TBB 0
#endif

#ifdef GTSAM_POINTS_USE_TBB
#define IS_DEFINED_GTSAM_POINTS_USE_TBB 1
#else
#define IS_DEFINED_GTSAM_POINTS_USE_TBB 0
#endif

#ifdef GTSAM_POINTS_USE_OPENMP
#define IS_DEFINED_GTSAM_POINTS_USE_OPENMP 1
#else
#define IS_DEFINED_GTSAM_POINTS_USE_OPENMP 0
#endif

#ifdef GTSAM_POINTS_USE_CUDA
#define IS_DEFINED_GTSAM_POINTS_USE_CUDA 1
#else
#define IS_DEFINED_GTSAM_POINTS_USE_CUDA 0
#endif

#ifdef GTSAM_POINTS_WITH_MARCH_NATIVE
#define IS_DEFINED_GTSAM_POINTS_WITH_MARCH_NATIVE 1
#else
#define IS_DEFINED_GTSAM_POINTS_WITH_MARCH_NATIVE 0
#endif

#define PRINT_IF_DEFINED(logger, name) logger->debug(#name " = {}", IS_DEFINED_##name)

namespace glim {

void print_system_info(std::shared_ptr<spdlog::logger> logger) {
  logger->debug("*** GTSAM ***");
  logger->debug("GTSAM_VERSION={}.{}.{} ({})", GTSAM_VERSION_MAJOR, GTSAM_VERSION_MINOR, GTSAM_VERSION_PATCH, GTSAM_VERSION_STRING);
  PRINT_IF_DEFINED(logger, GTSAM_USE_TBB);
  PRINT_IF_DEFINED(logger, GTSAM_ALLOCATOR_TBB);
  PRINT_IF_DEFINED(logger, GTSAM_USE_SYSTEM_EIGEN);
  PRINT_IF_DEFINED(logger, GTSAM_USE_SYSTEM_METIS);
  logger->debug("GTSAM_EIGEN_VERSION={}.{}", GTSAM_EIGEN_VERSION_WORLD, GTSAM_EIGEN_VERSION_MAJOR);

  logger->debug("*** GTSAM_POINTS ***");
  logger->debug("GTSAM_POINTS_VERSION={}.{}.{} ({})", GTSAM_POINTS_VERSION_MAJOR, GTSAM_POINTS_VERSION_MINOR, GTSAM_POINTS_VERSION_PATCH, GTSAM_POINTS_VERSION_STRING);
  logger->debug("GTSAM_POINTS_GIT_HASH={}", GTSAM_POINTS_GIT_HASH);
  PRINT_IF_DEFINED(logger, GTSAM_POINTS_USE_TBB);
  PRINT_IF_DEFINED(logger, GTSAM_POINTS_USE_OPENMP);
  PRINT_IF_DEFINED(logger, GTSAM_POINTS_USE_CUDA);
  PRINT_IF_DEFINED(logger, GTSAM_POINTS_WITH_MARCH_NATIVE);
  logger->debug("is_tbb_default = {}", gtsam_points::is_tbb_default());

#ifdef GTSAM_POINTS_USE_CUDA
  logger->debug("*** CUDA ***");
  logger->debug("GTSAM_POINTS_CUDA_VERSION={}.{}.{}", GTSAM_POINTS_CUDA_VERSION_MAJOR, GTSAM_POINTS_CUDA_VERSION_MINOR, GTSAM_POINTS_CUDA_VERSION_PATCH);
  const auto devices = gtsam_points::cuda_device_names();
  logger->debug("CUDA devices:");
  for (const auto& device : devices) {
    logger->debug("  - {}", device);
  }
#endif
}

}  // namespace glim