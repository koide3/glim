cmake_minimum_required(VERSION 3.22)
project(glim_trajectory_tests LANGUAGES CXX)

# This utility only depends on Eigen; testing it does not require ROS or CUDA.
find_package(Eigen3 REQUIRED)
add_executable(trajectory_manager_test
  trajectory_manager_test.cpp
  ../src/glim/util/trajectory_manager.cpp
)
target_compile_features(trajectory_manager_test PRIVATE cxx_std_17)
target_include_directories(trajectory_manager_test PRIVATE ../include)
target_link_libraries(trajectory_manager_test PRIVATE Eigen3::Eigen)
# Keep checked vector access enabled even in optimized GCC/Clang builds.
target_compile_definitions(trajectory_manager_test PRIVATE _GLIBCXX_ASSERTIONS)

enable_testing()
foreach(test_case before_first_odom future_anchor just_after_latest repeated_future_anchors
                  retry_after_odom exact_latest after_pruning interpolation)
  add_test(NAME trajectory_${test_case} COMMAND trajectory_manager_test ${test_case})
endforeach()
