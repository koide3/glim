
GLIM is tested on Ubuntu 20.04 with CUDA 11.6 / Ubuntu 22.04 with CUDA 11.8 / NVIDIA Jetson Xavier and Orin (JetPack 5.0.1).

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
# For Ubuntu 22.04, add -DGTSAM_USE_SYSTEM_EIGEN=ON
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_WITH_TBB=OFF \
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
```

### Installation for ROS1

```bash
cd ~/catkin_ws/src
git clone https://github.com/koide3/glim --recursive
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
git clone https://github.com/koide3/glim --recursive
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
    While you can enable AVX intrinsics to speed up the mapping system by setting ```BUILD_WITH_MARCH_NATIVE=ON```, it sometimes causes segfaults unless you properly set ```march=native``` for *every* involved library. We recommend keeping it disabled if you are not sure.
