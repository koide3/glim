# Docker images

## Prebuilt docker images

We provide the following docker images for ROS1 and ROS2 environments on docker hub.

- [![Docker Image Size (tag)](https://img.shields.io/docker/image-size/koide3/glim_ros1/noetic) koide3/glim_ros1:noetic](https://hub.docker.com/repository/docker/koide3/glim_ros1/tags)
- [![Docker Image Size (tag)](https://img.shields.io/docker/image-size/koide3/glim_ros1/noetic_cuda12.2) koide3/glim_ros1:noetic_cuda12.2](https://hub.docker.com/repository/docker/koide3/glim_ros1/tags)
- [![Docker Image Size (tag)](https://img.shields.io/docker/image-size/koide3/glim_ros2/jazzy) koide3/glim_ros2:jazzy](https://hub.docker.com/repository/docker/koide3/glim_ros2/tags)
- [![Docker Image Size (tag)](https://img.shields.io/docker/image-size/koide3/glim_ros2/humble) koide3/glim_ros2:humble](https://hub.docker.com/repository/docker/koide3/glim_ros2/tags)
- [![Docker Image Size (tag)](https://img.shields.io/docker/image-size/koide3/glim_ros2/humble_cuda12.2) koide3/glim_ros2:humble_cuda12.2](https://hub.docker.com/repository/docker/koide3/glim_ros2/tags)

Thirdparty images:
- [![Docker Image Size (tag)](https://img.shields.io/docker/image-size/junekyoopark/arm64v8_glim_ros1_cuda12.2/latest) ROS1 on Jetpack 5.1.4 (Jetson Orin NX)](https://hub.docker.com/r/junekyoopark/arm64v8_glim_ros1_cuda12.2) (made by [junekyoopark](https://github.com/junekyoopark))


!!! note
    ROS2 sometimes requires additional configurations for communication on docker. See [https://github.com/eProsima/Fast-DDS/issues/2956](https://github.com/eProsima/Fast-DDS/issues/2956). Do not ask us about how to use ROS2 with docker.

!!! note
    Currently, we provide only AMD64 images. ARM64 support is planned in the future.

### Example use

#### With GPU

```bash
# Copy config and edit as you want
git clone git@github.com:koide3/glim /tmp/glim
cp -R /tmp/glim/config ./config

# Pull image from docker hub
docker pull koide3/glim_ros2:humble_cuda12.2

# Launch glim_ros2:humble_cuda12.2 image with GPU and DISPLAY support
docker run \
  -it \
  --rm \
  --net=host \
  --ipc=host \
  --pid=host \
  --gpus all \
  -e=DISPLAY \
  -e=ROS_DOMAIN_ID \
  -v $(realpath config):/glim/config \
  koide3/glim_ros2:humble_cuda12.2 \
  ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/glim/config
```

#### Without GPU

```bash
# Copy config and edit it
git clone git@github.com:koide3/glim /tmp/glim
cp -R /tmp/glim/config ./config

# Change as follows:
# "config_odometry" : "config_odometry_cpu.json"
# "config_sub_mapping" : "config_sub_mapping_cpu.json"
# "config_global_mapping" : "config_global_mapping_cpu.json"
nano config/config.json

# Pull image from docker hub
docker pull koide3/glim_ros2:humble

# Launch glim_ros2:humble image with DISPLAY support
docker run \
  -it \
  --rm \
  --net=host \
  --ipc=host \
  --pid=host \
  --gpus all \
  -e=DISPLAY \
  -e=ROS_DOMAIN_ID \
  -v $(realpath config):/glim/config \
  koide3/glim_ros2:humble \
  ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/glim/config
```


## Build docker images from source

```
mkdir /tmp/glim_docker && cd /tmp/glim_docker
git clone git@github.com:koide3/glim
git clone git@github.com:koide3/glim_ros2

# Without GPU
docker build \
  -f glim_ros2/docker/Dockerfile.gcc \
  --build-arg="BASE_IMAGE=koide3/gtsam_points:jammy" \
  --build-arg="ROS_DISTRO=humble" \
  --tag glim_ros2:humble \
  .

# With GPU
docker build \
  -f glim_ros2/docker/Dockerfile.gcc.cuda \
  --build-arg="BASE_IMAGE=koide3/gtsam_points:jammy_cuda12.2" \
  --build-arg="ROS_DISTRO=humble" \
  --tag glim_ros2:humble_cuda12.2 \
  .
```