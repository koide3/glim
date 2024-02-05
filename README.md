![GLIM](docs/assets/logo2.png "GLIM Logo")

## Important Notes

Releases after v0.1.1 (239a75cf2bbc2dcc594dee3d801b005ae04a19b7, Nov, 27th, 2023) can be subject to the change to a non-permissive license. We have not decided anything on the new license model. Note that v0.1.1 and earlier remain on the MIT license.

## Introduction

**GLIM** is a versatile and extensible range-based 3D mapping framework.

- ***Accuracy:*** The backend of GLIM is based on global matching cost minimization that enables to accurately retain the global consistency of a map. Optionally, GPU acceleration can be used to maximize the mapping speed and quality.
- ***Easy-to-use:*** GLIM offers an interactive map correction interface that enables the user to manually correct mapping failures and easily refine mapping results.
- ***Versatility:*** As we eliminated sensor-specific processes, GLIM can be applied to any kind of range sensors including:
  - Spinning-type LiDAR (e.g., Velodyne HDL32e)
  - Non-repetitive scan LiDAR (e.g., Livox Avia)
  - Solid-state LiDAR (e.g., Intel Realsense L515)
  - RGB-D camera (e.g., Microsoft Azure Kinect)
- ***Extensibility:*** GLIM provides the global callback slot mechanism that allows to access the internal states of the mapping process and insert additional constraints to the factor graph. We also release [glim_ext](https://github.com/koide3/glim_ext) that offers several extension functions (e.g., explicit loop detection, LiDAR-Visual-Inertial frontend).

Tested on Ubuntu 20.04 with CUDA 11.6 / Ubuntu 22.04 with CUDA 11.8 / NVIDIA Jetson Xavier and Orin (JetPack 5.0.1).

[![Build](https://github.com/koide3/glim/actions/workflows/build.yml/badge.svg)](https://github.com/koide3/glim/actions/workflows/build.yml)
[![ROS1](https://github.com/koide3/glim_ros1/actions/workflows/docker_push.yml/badge.svg)](https://github.com/koide3/glim_ros1/actions/workflows/docker_push.yml)
[![ROS2](https://github.com/koide3/glim_ros2/actions/workflows/docker_push.yml/badge.svg)](https://github.com/koide3/glim_ros2/actions/workflows/docker_push.yml)

## Dependencies
### Mandatory
- [Eigen](https://eigen.tuxfamily.org/index.php)
- [nanoflann](https://github.com/jlblancoc/nanoflann)
- [OpenCV](https://opencv.org/)
- [GTSAM](https://github.com/borglab/gtsam)
- [gtsam_ext](https://github.com/koide3/gtsam_ext)

### Optional
- [CUDA](https://developer.nvidia.com/cuda-toolkit)
- [OpenMP](https://www.openmp.org/)
- [ROS/ROS2](https://www.ros.org/)
- [Iridescence](https://github.com/koide3/iridescence) (for visualization)
- [glim_ext](https://github.com/koide3/glim_ext) (for extension functions)

## License

For commercial support, please contact ```k.koide@aist.go.jp```.

## Contact
[Kenji Koide](https://staff.aist.go.jp/k.koide/), k.koide@aist.go.jp  
National Institute of Advanced Industrial Science and Technology (AIST), Japan

