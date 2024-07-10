![GLIM](docs/assets/logo2.png "GLIM Logo")

## Introduction

**GLIM** is a versatile and extensible range-based 3D mapping framework.

- ***Accuracy:*** GLIM is based on direct multi-scan registration error minimization on factor graphs that enables to accurately retain the consistency of mappint results. GPU acceleration is supported to maximize the mapping speed and quality.
- ***Easy-to-use:*** GLIM offers an interactive map correction interface that enables the user to manually correct mapping failures and easily refine mapping results.
- ***Versatility:*** As we eliminated sensor-specific processes, GLIM can be applied to any kind of range sensors including:
    - Spinning-type LiDAR (e.g., Velodyne HDL32e)
    - Non-repetitive scan LiDAR (e.g., Livox Avia)
    - Solid-state LiDAR (e.g., Intel Realsense L515)
    - RGB-D camera (e.g., Microsoft Azure Kinect)
- ***Extensibility:*** GLIM provides the global callback slot mechanism that allows to access the internal states of the mapping process and insert additional constraints to the factor graph. We also release [glim_ext](https://github.com/koide3/glim_ext) that offers example implementations of several extension functions (e.g., explicit loop detection, LiDAR-Visual-Inertial odometry estimation).

Tested on Ubuntu 22.04 /24.04 with CUDA 12.2, and NVIDIA Jetson Orin.

[![Build](https://github.com/koide3/glim/actions/workflows/build.yml/badge.svg)](https://github.com/koide3/glim/actions/workflows/build.yml)
[![ROS1](https://github.com/koide3/glim_ros1/actions/workflows/docker_push.yml/badge.svg)](https://github.com/koide3/glim_ros1/actions/workflows/docker_push.yml)
[![ROS2](https://github.com/koide3/glim_ros2/actions/workflows/docker_push.yml/badge.svg)](https://github.com/koide3/glim_ros2/actions/workflows/docker_push.yml)

## Dependencies
### Mandatory
- [Eigen](https://eigen.tuxfamily.org/index.php)
- [nanoflann](https://github.com/jlblancoc/nanoflann)
- [OpenCV](https://opencv.org/)
- [GTSAM](https://github.com/borglab/gtsam)
- [gtsam_points](https://github.com/koide3/gtsam_points)

### Optional
- [CUDA](https://developer.nvidia.com/cuda-toolkit)
- [OpenMP](https://www.openmp.org/)
- [ROS/ROS2](https://www.ros.org/)
- [Iridescence](https://github.com/koide3/iridescence) (for visualization)
- [glim_ext](https://github.com/koide3/glim_ext) (for extension functions)

## License

This package is released under the MIT license. For commercial support, please contact ```k.koide@aist.go.jp```.

## Contact
[Kenji Koide](https://staff.aist.go.jp/k.koide/), k.koide@aist.go.jp<br>
National Institute of Advanced Industrial Science and Technology (AIST), Japan

