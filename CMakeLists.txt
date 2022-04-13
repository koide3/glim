cmake_minimum_required(VERSION 3.5)
project(glim)

add_compile_options(-std=c++17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

option(BUILD_WITH_CUDA "Build with GPU support" ON)
option(BUILD_WITH_VIEWER "Build with visualizer" ON)
option(BUILD_WITH_MARCH_NATIVE "Build with -march=native" OFF)

if(BUILD_WITH_MARCH_NATIVE)
  add_definitions(-march=native)
  set(CMAKE_C_FLAGS "-march=native ${CMAKE_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "-march=native ${CMAKE_CXX_FLAGS}")
endif()

find_package(Boost REQUIRED timer serialization)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED core)
find_package(GTSAM REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(thirdparty/json)
add_subdirectory(thirdparty/gtsam_ext)

###########
## Build ##
###########

# GPU-related
if(BUILD_WITH_CUDA)
  add_definitions(-DBUILD_GTSAM_EXT_GPU)
  install(TARGETS gtsam_ext_cuda)
  list(APPEND GPU_SRCS src/glim/frontend/odometry_estimation_gpu.cpp)
  list(APPEND glim_LIBRARIES gtsam_ext_cuda)
endif()

add_library(glim SHARED
  # util
  src/glim/util/config.cpp
  src/glim/util/export_factors.cpp
  src/glim/util/time_keeper.cpp
  # preprocess
  src/glim/preprocess/cloud_preprocessor.cpp
  src/glim/preprocess/downsampling.cpp
  # common process
  src/glim/common/callbacks.cpp
  src/glim/common/imu_integration.cpp
  src/glim/common/cloud_deskewing.cpp
  src/glim/common/cloud_covariance_estimation.cpp
  # frontend
  src/glim/frontend/callbacks.cpp
  src/glim/frontend/initial_state_estimation.cpp
  src/glim/frontend/odometry_estimation_base.cpp
  src/glim/frontend/async_odometry_estimation.cpp
  src/glim/frontend/odometry_estimation_ct.cpp
  # backend
  src/glim/backend/callbacks.cpp
  src/glim/backend/sub_map.cpp
  src/glim/backend/sub_mapping_base.cpp
  src/glim/backend/global_mapping_base.cpp
  src/glim/backend/async_sub_mapping.cpp
  src/glim/backend/async_global_mapping.cpp
  src/glim/backend/sub_mapping.cpp
  src/glim/backend/global_mapping.cpp
  # GPU-related
  ${GPU_SRCS}
)
target_include_directories(glim PUBLIC
  include
  thirdparty/json/include
  thirdparty/gtsam_ext/include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
)
target_link_libraries(glim
  gtsam_ext
  ${OpenCV_LIBRARIES}
  ${GTSAM_LIBRARIES}
)


list(APPEND glim_LIBRARIES glim gtsam_ext)
install(TARGETS glim gtsam_ext)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY thirdparty/gtsam_ext/include/ DESTINATION include)

if(BUILD_WITH_VIEWER)
  find_package(Iridescence REQUIRED)

  add_library(glim_vis SHARED
    src/glim/viewer/standard_viewer.cpp
    src/glim/viewer/interactive_viewer.cpp
    src/glim/viewer/interactive/manual_loop_close_modal.cpp
    src/glim/viewer/interactive/bundle_adjustment_modal.cpp
    src/glim/viewer/offline_viewer.cpp
  )
  target_include_directories(glim_vis PUBLIC
    include
    thirdparty/json/include
    thirdparty/gtsam_ext/include
    ${Iridescence_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIRS}
  )
  target_link_libraries(glim_vis
    ${Iridescence_LIBRARIES}
  )

  list(APPEND glim_LIBRARIES glim_vis)
  install(TARGETS glim_vis)
endif()

#################
## ROS-related ##
#################
if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 2)
    # ROS2
    find_package(ament_cmake REQUIRED)
    install(DIRECTORY config DESTINATION share/glim)
    ament_export_include_directories(include)
    ament_export_libraries(${glim_LIBRARIES})
    ament_package()
  elseif($ENV{ROS_VERSION} EQUAL 1)
    # ROS1
    find_package(catkin REQUIRED)
    catkin_package(
      INCLUDE_DIRS include thirdparty/gtsam_ext/include
      LIBRARIES ${glim_LIBRARIES}
    )
  endif()
endif()