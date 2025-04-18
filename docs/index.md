# Home

## Introduction

![GLIM](assets/logo2.png "GLIM Logo")

**GLIM** is a versatile and extensible range-based 3D mapping framework.

- ***Accuracy:*** GLIM is based on direct multi-scan registration error minimization on factor graphs that enables to accurately retain the consistency of mappint results. GPU acceleration is supported to maximize the mapping speed and quality.
- ***Easy-to-use:*** GLIM offers an interactive map correction interface that enables the user to manually correct mapping failures and easily refine mapping results.
- ***Versatility:*** As we eliminated sensor-specific processes, GLIM can be applied to any kind of range sensors including:
    - Spinning-type LiDAR (e.g., Velodyne HDL32e)
    - Non-repetitive scan LiDAR (e.g., Livox Avia)
    - Solid-state LiDAR (e.g., Intel Realsense L515)
    - RGB-D camera (e.g., Microsoft Azure Kinect)
- ***Extensibility:*** GLIM provides the global callback slot mechanism that allows to access the internal states of the mapping process and insert additional constraints to the factor graph. We also release [glim_ext](https://github.com/koide3/glim_ext) that offers example implementations of several extension functions (e.g., explicit loop detection, LiDAR-Visual-Inertial odometry estimation).

Tested on Ubuntu 22.04 / 24.04 with CUDA 12.2 / 12.5 / 12.6, and NVIDIA Jetson Orin (Jetpack 6.1).

[![Build test status](assets/build.svg)](https://github.com/koide3/glim/actions/workflows/build.yml)
[![ROS1](assets/ros1.svg)](https://github.com/koide3/glim_ros1/actions/workflows/docker_push.yml)
[![ROS2](assets/ros2.svg)](https://github.com/koide3/glim_ros2/actions/workflows/docker_push.yml)

## Video

### Robustness test
<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/Kk-K2rCXt-U" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

### Mapping with various range sensors

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/_fwK4awbW18?si=R5m5502i7sKTbopg" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>

### Outdoor driving test with Livox MID360

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/CIfRqeV0irE?si=WT-knUxMuGWjYcxQ" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>

### Merging Multiple Mapping Sessions

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/aMq3qbAgTeI?si=QRZqp0DjSK79NcQk" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>

### Manual Object Removal

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/FSkNsVNoCU4?si=MbCYOm-z9gbB_bbd" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>

See more in [Extension modules](extensions.md) and [Demo](demo.md) pages.

## Contact

Kenji Koide [:material-home:](https://staff.aist.go.jp/k.koide/) [:material-mail:](mailto:k.koide@aist.go.jp) [:material-twitter:](https://twitter.com/k_koide3)  
National Institute of Advanced Industrial Science and Technology (AIST), Japan