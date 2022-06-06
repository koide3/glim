![GLIM](docs/logo2.png "GLIM Logo")

**GLIM** is a versatile and extensible range-based 3D mapping framework.

- ***Accuracy:*** The backend of GLIM is based on global matching cost minimization that enables to accurately retain the global consistency of a map. Optionally, GPU acceleration can be used to maximize the mapping speed and quality.
- ***Easy-to-use:*** GLIM offers an interactive map correction interface that enables the user to manually correct mapping failures and easily refine mapping results. It also provides a self-tuning mechanism that relieves the user from tedious parameter tuning.
- ***Versatility:*** As we eliminated sensor-specific processes, GLIM can be applied to any kind of range sensors including:
  - Spinning-type LiDAR (e.g., Velodyne HDL32e)
  - Non-repetitive scan LiDAR (e.g., Livox Avia)
  - Solid-state LiDAR (e.g., Intel Realsense L515)
  - RGB-D camera (e.g., Microsoft Azure Kinect)
- ***Extensibility:*** GLIM provides the global callback slot mechanism that allows to access the internal states of the mapping process and insert additional constraints to the factor graph. We also release [glim_ext](https://github.com/SMRT-AIST/glim_ext) that offers several extension functions (e.g., explicit loop detection, LiDAR-Visual-Inertial frontend).

Tested on Ubuntu 20.04 with CUDA 11.6 / NVIDIA Jetson Xavier (JetPack 5.0.1).

[![Build](https://github.com/koide3/glim/actions/workflows/build.yml/badge.svg)](https://github.com/koide3/glim/actions/workflows/build.yml)

## Dependencies
### Mandatory
- [Eigen](https://eigen.tuxfamily.org/index.php)
- [nanoflann](https://github.com/jlblancoc/nanoflann)
- [OpenCV](https://opencv.org/)
- [GTSAM](https://github.com/borglab/gtsam)
- [gtsam_ext]()

### Optional
- [CUDA](https://developer.nvidia.com/cuda-toolkit)
- [OpenMP](https://www.openmp.org/)
- [ROS/ROS2](https://www.ros.org/)
- [Iridescence](https://github.com/koide3/iridescence) (for visualization)
- [glim_ext](https://github.com/koide3/glim_ext) (for extension functions)

## Installation
### Common dependencies

```bash
# Install libraries
sudo apt install libomp-dev libboost-all-dev

# Install GTSAM
git clone https://github.com/borglab/gtsam
mkdir gtsam/build && cd gtsam/build
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_WITH_TBB=OFF \
         -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF

make -j$(nproc)
sudo make install

# Install Iridescence for visualization
# This is optional but highly recommended
sudo apt install -y libglm-dev libglfw3-dev libpng-dev

git clone https://github.com/koide3/iridescence --recursive
mkdir iridescence/build && cd iridescence/build
cmake .. -DCMAKE_BUILD_TYPE=Release

make -j$(nproc)
sudo make install
```

### Installation for ROS1

```bash
cd ~/catkin_ws/src
git clone https://github.com/SMRT-AIST/glim --recursive
git clone https://github.com/SMRT-AIST/glim_ros1

# Optional extension library
git clone https://github.com/SMRT-AIST/glim_ext --recursive

cd ~/catkin_ws
catkin_make
```

### Installation for ROS2

```bash
cd ~/ros2_ws/src
git clone https://github.com/SMRT-AIST/glim --recursive
git clone https://github.com/SMRT-AIST/glim_ros

# Optional extension library
git clone https://github.com/SMRT-AIST/glim_ext --recursive

cd ~/ros2_ws
colcon_build
```

## License

This package is released under the GPLv3 license. For commercial purposes, please contact ```k.koide@aist.go.jp```.


## Papers
- Koide et al., "Voxelized GICP for Fast and Accurate 3D Point Cloud Registration", ICRA2021 [[link]](https://staff.aist.go.jp/k.koide/assets/pdf/icra2021_02.pdf)
- Koide et al., "Globally Consistent 3D LiDAR Mapping with GPU-accelerated GICP Matching Cost Factors", IEEE RA-L, 2021 [[link]](https://staff.aist.go.jp/k.koide/assets/pdf/ral2021.pdf)
- Koide et al., "Globally Consistent and Tightly Coupled 3D LiDAR Inertial Mapping", ICRA2022 [[link]](https://staff.aist.go.jp/k.koide/assets/pdf/icra2022.pdf)


## Contact
[Kenji Koide](https://staff.aist.go.jp/k.koide/), k.koide@aist.go.jp

[Mobile Robotics Research Team (MR2T)](https://unit.aist.go.jp/hcmrc/mr-rt/index.html), National Institute of Advanced Industrial Science and Technology (AIST), Japan

