# Required image for building gtsam_points with CUDA enabled
FROM nvidia/cuda:12.6.1-devel-ubuntu22.04

# Base images for JetPack 5 and JetPack 6
# FROM nvcr.io/nvidia/l4t-base:r35.4.1
# FROM nvcr.io/nvidia/l4t-jetpack:r36.3

# Set noninteractive to avoid prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Set a working directory
WORKDIR /workspace

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    cmake \
    libomp-dev \
    libboost-all-dev \
    libmetis-dev \
    libfmt-dev \
    libspdlog-dev \
    libglm-dev \
    libglfw3-dev \
    libpng-dev \
    libjpeg-dev \
    libeigen3-dev \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Install GTSAM
RUN git clone https://github.com/borglab/gtsam.git && \
    cd gtsam && \
    git checkout 4.3a0 && \
    mkdir build && \
    cd build && \
    cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
             -DGTSAM_BUILD_TESTS=OFF \
             -DGTSAM_WITH_TBB=OFF \
             -DGTSAM_USE_SYSTEM_EIGEN=ON \
             -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF && \
    make -j$(nproc) && \
    make install

# Install Iridescence for visualization
RUN git clone https://github.com/koide3/iridescence --recursive && \
    cd iridescence && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install

# Install gtsam_points
RUN git clone https://github.com/koide3/gtsam_points.git && \
    cd gtsam_points && \
    mkdir build && \
    cd build && \
    cmake .. -DBUILD_WITH_CUDA=ON && \
    make -j$(nproc) && \
    make install

# Make shared libraries visible to the system
RUN ldconfig

# Copy project source code into the container
COPY . /app
WORKDIR /app

# Create a build directory and build project with specified options
RUN mkdir build && cd build && \
    cmake .. \
      # On = GPU Off = CPU
      -DBUILD_WITH_CUDA=ON \
      # On = Compile with viewer
      -DBUILD_WITH_VIEWER=ON \
      # Off = Disable CPU optimization for older CPUs
      -DBUILD_WITH_MARCH_NATIVE=OFF \
      # Release = Standard
      -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install

# Install ROS2 and GLIM_ROS2 to use ros_2_bag_viewer. Comment out if not in use.
# ===================================================================================
SHELL ["/bin/bash", "-c"]

# Setup glim package files
RUN mkdir -p /usr/local/share/glim/ && \
    cp /app/package.xml /usr/local/share/glim/ && \
    mkdir -p /usr/local/share/ament_index/resource_index/packages && \
    touch /usr/local/share/ament_index/resource_index/packages/glim

# Install ROS 2 and all dependencies
RUN apt-get update && \
    apt-get install -y curl lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-humble-desktop \
      ros-humble-rosbag2-storage-mcap \
      python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

# Create ROS 2 workspace directory
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src
RUN git clone https://github.com/koide3/glim_ros2.git

WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.bash && \
    export AMENT_PREFIX_PATH=/usr/local:$AMENT_PREFIX_PATH && \
    colcon build --symlink-install

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export AMENT_PREFIX_PATH=/usr/local:\$AMENT_PREFIX_PATH" >> /root/.bashrc
# ===================================================================================

# Set the final working directory
WORKDIR /app/build

# Default command to keep the container running
CMD ["tail", "-f", "/dev/null"]


# Commands for testing
# xhost +local:
# docker exec -it glim_container bash
# ros2 run glim_ros glim_rosbag ros_bags/