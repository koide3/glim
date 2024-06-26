# Home

Code will be made public on github. API lists are available at [gtsam_points](https://staff.aist.go.jp/k.koide/projects/doxygen/gtsam_points/) / [glim](https://staff.aist.go.jp/k.koide/projects/doxygen/glim/) / [glim_ext](https://staff.aist.go.jp/k.koide/projects/doxygen/glim_ext/).

## Introduction

![GLIM](assets/logo2.png "GLIM Logo")

**GLIM** is a versatile and extensible range-based 3D mapping framework.

- ***Accuracy:*** The loop closure of GLIM is performed based on global matching cost minimization that enables to accurately retain the global consistency of a map. Optionally, GPU acceleration can be used to maximize the mapping speed and quality.
- ***Easy-to-use:*** GLIM offers an interactive map correction interface that enables the user to manually correct mapping failures and easily refine mapping results.
- ***Versatility:*** As we eliminated sensor-specific processes, GLIM can be applied to any kind of range sensors including:
    - Spinning-type LiDAR (e.g., Velodyne HDL32e)
    - Non-repetitive scan LiDAR (e.g., Livox Avia)
    - Solid-state LiDAR (e.g., Intel Realsense L515)
    - RGB-D camera (e.g., Microsoft Azure Kinect)
- ***Extensibility:*** GLIM provides the global callback slot mechanism that allows to access the internal states of the mapping process and insert additional constraints to the factor graph. We also release [glim_ext](https://github.com/koide3/glim_ext) that offers several extension functions (e.g., explicit loop detection, LiDAR-Visual-Inertial odometry estimation).

Tested on Ubuntu 20.04 with CUDA 11.6 / Ubuntu 22.04 with CUDA 11.8 / NVIDIA Jetson Xavier and Orin (JetPack 5.0.1).

[![Build test status](assets/build.svg)](https://github.com/koide3/glim/actions/workflows/build.yml)
[![ROS1](assets/ros1.svg)](https://github.com/koide3/glim_ros1/actions/workflows/docker_push.yml)
[![ROS2](assets/ros2.svg)](https://github.com/koide3/glim_ros2/actions/workflows/docker_push.yml)

## API List

* **[gtsam_points](https://staff.aist.go.jp/k.koide/projects/doxygen/gtsam_points/):** GTSAM factors for range-based SLAM (MIT License)
* **[glim](https://staff.aist.go.jp/k.koide/projects/doxygen/glim/):** Main LiDAR-IMU SLAM package (GPLv3)
* **[glim_ext](https://staff.aist.go.jp/k.koide/projects/doxygen/glim_ext/):** Example implementations of GLIM extension modules (GPLv3)

## Video

### Robustness test
<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/Kk-K2rCXt-U" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

### Mapping with various range sensors

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/rLqYo42eDTQ" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

See more in [Extension modules](extensions.md) and [Demo](demo.md) pages.

## Contact

Kenji Koide [:material-home:](https://staff.aist.go.jp/k.koide/) [:material-mail:](mailto:k.koide@aist.go.jp) [:material-twitter:](https://twitter.com/k_koide3)  
National Institute of Advanced Industrial Science and Technology (AIST), Japan