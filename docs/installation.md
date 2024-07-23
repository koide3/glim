
GLIM is tested on Ubuntu 22.04 / 24.04 with CUDA 12.2, and NVIDIA Jetson Orin (JetPack 6.0).

## Install from source

### Common dependencies
```bash
# Install dependencies
sudo apt install libomp-dev libboost-all-dev libmetis-dev \
                 libfmt-dev libspdlog-dev \
                 libglm-dev libglfw3-dev libpng-dev libjpeg-dev

# Install GTSAM
git clone https://github.com/borglab/gtsam
cd gtsam && git checkout 4.2a9
mkdir build && cd build
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_WITH_TBB=OFF \
         -DGTSAM_USE_SYSTEM_EIGEN=ON \
         -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
make -j$(nproc)
sudo make install

# Install Iridescence for visualization
# This is optional but highly recommended
git clone https://github.com/koide3/iridescence --recursive
mkdir iridescence/build && cd iridescence/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install


# Install gtsam_points
git clone https://github.com/koide3/gtsam_points
mkdir gtsam_points/build && cd gtsam_points/build
cmake .. -DBUILD_WITH_CUDA=ON
make -j$(nproc)
sudo make install


# Make shared libraries visible to the system
sudo ldconfig
```

### Installation for ROS1

```bash
cd ~/catkin_ws/src
git clone https://github.com/koide3/glim
git clone https://github.com/koide3/glim_ros1

cd ~/catkin_ws
catkin_make

# cmake options
# catkin_make \
#   -DBUILD_WITH_CUDA=ON \
#   -DBUILD_WITH_VIEWER=ON \
#   -DBUILD_WITH_MARCH_NATIVE=OFF
```

### Installation for ROS2
```bash
cd ~/ros2_ws/src
git clone https://github.com/koide3/glim
git clone https://github.com/koide3/glim_ros2

cd ~/ros2_ws
colcon build

# cmake options
# colcon build --cmake-args \
#   -DBUILD_WITH_CUDA=ON \
#   -DBUILD_WITH_VIEWER=ON \
#   -DBUILD_WITH_MARCH_NATIVE=OFF
```

!!! note
    While AVX intrinsics can be enabled to speed up the mapping process by setting ```BUILD_WITH_MARCH_NATIVE=ON```, it sometimes causes segfaults unless ```march=native``` is properly set for *every* involved library. We recommend keeping it disabled if you are not sure.
