cmake_minimum_required(VERSION 3.16)
project(glim VERSION 1.1.0 LANGUAGES CXX)

add_compile_options(-std=c++17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

option(BUILD_WITH_CUDA "Build with GPU support" ON)
option(BUILD_WITH_CUDA_MULTIARCH "Build with GPU cross-platform support" OFF)
option(BUILD_WITH_VIEWER "Build with visualizer" ON)
option(BUILD_WITH_MARCH_NATIVE "Build with -march=native" OFF)
option(BUILD_WITH_OPENCV "Build with OpenCV to support camera images" ON)

if(BUILD_WITH_MARCH_NATIVE)
  add_definitions(-march=native)
  add_definitions(-DBUILD_WITH_MARCH_NATIVE)
  set(CMAKE_C_FLAGS "-march=native ${CMAKE_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "-march=native ${CMAKE_CXX_FLAGS}")
endif()

find_package(Boost REQUIRED serialization)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(gtsam_points REQUIRED)
find_package(OpenMP REQUIRED)
find_package(spdlog REQUIRED)

if(BUILD_WITH_VIEWER)
  find_package(Iridescence REQUIRED)
endif()

if(BUILD_WITH_OPENCV)
  set(GLIM_USE_OPENCV 1)
  find_package(OpenCV REQUIRED COMPONENTS core)
else()
  set(GLIM_USE_OPENCV 0)
endif()

###########
## Build ##
###########

# GPU-related
if(BUILD_WITH_CUDA)
  if(NOT GTSAM_POINTS_USE_CUDA)
    message(WARNING "gtsam_points is not built with CUDA!!")
    set(BUILD_WITH_CUDA OFF CACHE BOOL "gtsam_points is not built with CUDA" FORCE)
  else()
    set(GPU_SRCS src/glim/odometry/odometry_estimation_gpu.cpp)
  endif()
endif()

add_library(glim SHARED
  # util
  src/glim/util/config.cpp
  src/glim/util/logging.cpp
  src/glim/util/export_factors.cpp
  src/glim/util/time_keeper.cpp
  src/glim/util/trajectory_manager.cpp
  src/glim/util/extension_module.cpp
  src/glim/util/load_module.cpp
  src/glim/util/data_validator.cpp
  src/glim/util/serialization.cpp
  src/glim/util/debug.cpp
  # preprocess
  src/glim/preprocess/callbacks.cpp
  src/glim/preprocess/cloud_preprocessor.cpp
  # common process
  src/glim/common/imu_integration.cpp
  src/glim/common/cloud_deskewing.cpp
  src/glim/common/cloud_covariance_estimation.cpp
  # odometry
  src/glim/odometry/callbacks.cpp
  src/glim/odometry/estimation_frame.cpp
  src/glim/odometry/initial_state_estimation.cpp
  src/glim/odometry/loose_initial_state_estimation.cpp
  src/glim/odometry/odometry_estimation_base.cpp
  src/glim/odometry/async_odometry_estimation.cpp
  src/glim/odometry/odometry_estimation_imu.cpp
  src/glim/odometry/odometry_estimation_ct.cpp
  src/glim/odometry/odometry_estimation_cpu.cpp
  # mapping
  src/glim/mapping/callbacks.cpp
  src/glim/mapping/sub_map.cpp
  src/glim/mapping/sub_mapping_base.cpp
  src/glim/mapping/global_mapping_base.cpp
  src/glim/mapping/async_sub_mapping.cpp
  src/glim/mapping/async_global_mapping.cpp
  src/glim/mapping/sub_mapping.cpp
  src/glim/mapping/sub_mapping_passthrough.cpp
  src/glim/mapping/global_mapping.cpp
  src/glim/mapping/global_mapping_pose_graph.cpp
  # GPU-related
  ${GPU_SRCS}
)
target_include_directories(glim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/json/include>
  $<INSTALL_INTERFACE:include/glim>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(glim
  Boost::boost
  Boost::serialization
  Eigen3::Eigen
  gtsam
  gtsam_points::gtsam_points
  OpenMP::OpenMP_CXX
  spdlog::spdlog
  $<TARGET_NAME_IF_EXISTS:Iridescence::Iridescence>
  ${OpenCV_LIBRARIES}
)
if(BUILD_WITH_OPENCV)
  target_compile_definitions(glim PUBLIC GLIM_USE_OPENCV)
endif()
list(APPEND glim_LIBRARIES glim)

# Create shared libraries for estimation modules
list(APPEND module_srcs
  src/glim/odometry/odometry_estimation_ct_create.cpp
  src/glim/odometry/odometry_estimation_cpu_create.cpp
  src/glim/mapping/sub_mapping_create.cpp
  src/glim/mapping/sub_mapping_passthrough_create.cpp
  src/glim/mapping/global_mapping_create.cpp
  src/glim/mapping/global_mapping_pose_graph_create.cpp
)
if(BUILD_WITH_CUDA)
  list(APPEND module_srcs
    src/glim/odometry/odometry_estimation_gpu_create.cpp
  )
endif()

foreach(module_src IN LISTS module_srcs)
  get_filename_component(module_src_filename ${module_src} NAME_WE)
  string(REGEX REPLACE "_create" "" module_name ${module_src_filename})

  add_library(${module_name} SHARED
    ${module_src}
  )
  target_link_libraries(${module_name}
    glim
  )
  list(APPEND glim_LIBRARIES ${module_name})
endforeach()


if(BUILD_WITH_VIEWER)
  # Standard viewer
  add_library(standard_viewer SHARED
    src/glim/viewer/standard_viewer.cpp
  )
  target_link_libraries(standard_viewer
    glim
  )

  # Interactive viewer
  add_library(interactive_viewer SHARED
    src/glim/viewer/interactive_viewer.cpp
    src/glim/viewer/interactive/manual_loop_close_modal.cpp
    src/glim/viewer/interactive/bundle_adjustment_modal.cpp
    src/glim/viewer/offline_viewer.cpp
  )
  target_link_libraries(interactive_viewer
    glim
  )

  # Map editor
  add_library(map_editor SHARED
    src/glim/viewer/editor/map_cell.cpp
    src/glim/viewer/editor/points_selector.cpp
    src/glim/viewer/map_editor.cpp
  )
  target_link_libraries(map_editor
    glim
  )
  
  # Memory monitor
  add_library(memory_monitor SHARED
    src/glim/viewer/memory_monitor.cpp
  )
  target_link_libraries(memory_monitor
    glim
  )

  list(APPEND glim_LIBRARIES standard_viewer interactive_viewer memory_monitor map_editor)
endif()

#############
## Install ##
#############

if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 1)
    # No idea
    set(CMAKE_INSTALL_INCLUDEDIR "include/glim")
    set(CMAKE_INSTALL_LIBDIR "lib")
    set(CMAKE_INSTALL_BINDIR "bin/glim")
    set(CMAKE_INSTALL_DATADIR "share/glim")
    set(CMAKE_CONFIG_INSTALL_DIR "share/glim")
  elseif($ENV{ROS_VERSION} EQUAL 2)
    set(CMAKE_INSTALL_INCLUDEDIR "include/glim")
    set(CMAKE_INSTALL_LIBDIR "lib")
    set(CMAKE_INSTALL_BINDIR "bin/glim")
    set(CMAKE_INSTALL_DATADIR "share/glim")
    set(CMAKE_CONFIG_INSTALL_DIR "share/glim")
  else()
    message(FATAL_ERROR "Unsupported ROS version")
  endif()
else()
  include(GNUInstallDirs)
  set(CMAKE_CONFIG_INSTALL_DIR
    "${CMAKE_INSTALL_LIBDIR}/cmake/glim"
    CACHE PATH "Install directory for CMake config files"
  )
endif()

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS ${glim_LIBRARIES}
  EXPORT glim-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
include(CMakePackageConfigHelpers)
install(EXPORT glim-targets
  FILE glim-targets.cmake
  NAMESPACE glim::
  DESTINATION ${CMAKE_CONFIG_INSTALL_DIR}
)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/glim-config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/glim-config.cmake"
  INSTALL_DESTINATION ${CMAKE_CONFIG_INSTALL_DIR}
)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/glim-config-version.cmake"
  VERSION ${VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/glim-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/glim-config-version.cmake"
  DESTINATION ${CMAKE_CONFIG_INSTALL_DIR}
)

#################
## ROS-related ##
#################
if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 2)
    # ROS2
    find_package(ament_cmake REQUIRED)
    install(DIRECTORY config DESTINATION share/glim)
    ament_export_include_directories(include ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
    ament_export_libraries(${glim_LIBRARIES} ${GTSAM_LIBRARIES} fmt spdlog::spdlog)
    ament_export_targets(glim-targets HAS_LIBRARY_TARGET)
    ament_package(CONFIG_EXTRAS "cmake/glim-config.cmake.in")
  elseif($ENV{ROS_VERSION} EQUAL 1)
    # ROS1
    find_package(catkin REQUIRED)
    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ${glim_LIBRARIES} ${GTSAM_LIBRARIES} fmt spdlog::spdlog
    )
    install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  endif()
endif()
