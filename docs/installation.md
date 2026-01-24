
GLIM is tested on Ubuntu 22.04 / 24.04 with CUDA 12.2 / 12.6 / 13.1, and NVIDIA Jetson Orin (JetPack 6.1). You can build and install GLIM from source code, or install pre-built binaries from PPA.

## Install from [PPA](https://koide3.github.io/ppa/) [Ubuntu 24.04, 22.04] [AMD64, ARM64]

### Prerequisite

```bash
sudo apt install curl gpg
```

### Setup PPA

```bash
# Automatically setup PPA via online script
curl -s https://koide3.github.io/ppa/setup_ppa.sh | sudo bash
```

<details>
<summary>Manually setup PPA (If you don't want to use the online script)</summary>
```bash
# Manually setup PPA for Ubuntu 24.04
curl -s --compressed "https://koide3.github.io/ppa/ubuntu2404/KEY.gpg" | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/koide3_ppa.gpg >/dev/null
echo "deb [signed-by=/etc/apt/trusted.gpg.d/koide3_ppa.gpg] https://koide3.github.io/ppa/ubuntu2404 ./" | sudo tee /etc/apt/sources.list.d/koide3_ppa.list
sudo apt update

# Manually setup PPA for Ubuntu 22.04
curl -s --compressed "https://koide3.github.io/ppa/ubuntu2204/KEY.gpg" | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/koide3_ppa.gpg >/dev/null
echo "deb [signed-by=/etc/apt/trusted.gpg.d/koide3_ppa.gpg] https://koide3.github.io/ppa/ubuntu2204 ./" | sudo tee /etc/apt/sources.list.d/koide3_ppa.list
sudo apt update
```
</details>

### Install dependencies

```bash
sudo apt update
sudo apt install -y libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev

# Choose one of the follows
sudo apt install -y libgtsam-points-dev           # without CUDA
sudo apt install -y libgtsam-points-cuda12.2-dev  # with CUDA 12.2 (Ubuntu 22.04 only)
sudo apt install -y libgtsam-points-cuda12.6-dev  # with CUDA 12.6
sudo apt install -y libgtsam-points-cuda13.1-dev  # with CUDA 13.1
```

### Install GLIM for ROS2

```bash
# Choose one of the follows

# ROS2 jazzy (Ubuntu 24.04)
sudo apt install -y ros-jazzy-glim-ros             # Without CUDA
sudo apt install -y ros-jazzy-glim-ros-cuda12.6    # With CUDA 12.6
sudo apt install -y ros-jazzy-glim-ros-cuda13.1    # With CUDA 13.1

# ROS2 humble (Ubuntu 22.04)
sudo apt install -y ros-humble-glim-ros            # Without CUDA
sudo apt install -y ros-humble-glim-ros-cuda12.2   # With CUDA 12.2
sudo apt install -y ros-humble-glim-ros-cuda12.6   # With CUDA 12.6
sudo apt install -y ros-humble-glim-ros-cuda13.1   # With CUDA 13.1
```

```bash
# After installation, make shared libraries visible to the system
sudo ldconfig
```


## Install from source

### Common dependencies

```bash
# Install dependencies
sudo apt install libomp-dev libboost-all-dev libmetis-dev \
                 libfmt-dev libspdlog-dev \
                 libglm-dev libglfw3-dev libpng-dev libjpeg-dev

# Install GTSAM
git clone https://github.com/borglab/gtsam
cd gtsam && git checkout 4.3a0
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

### Install GLIM for ROS2
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



