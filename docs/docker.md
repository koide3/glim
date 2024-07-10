# Docker images

!!!warning
    Docker support is suspended now.

## Prebuilt docker images

We provide the following docker images for ROS1 and ROS2 environments on docker hub ([koide3/glim_ros](https://hub.docker.com/repository/docker/koide3/glim_ros)).

- koide3/glim_ros:noetic
- koide3/glim_ros:noetic_cuda12.2
- koide3/glim_ros:humble
- koide3/glim_ros:humble_cuda12.2
- koide3/glim_ros:jazzy

### Example use

```bash
# Pull image from docker hub
docker pull koide3/glim_ros:humble_cuda11.2

# Launch glim_ros:noetic image with GPU and X11 support
docker run \
  --net host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  koide3/glim_ros:humble_cuda11.2 \
  ros2 run glim_ros glim_rosnode
```

!!! note
    Currently, we provide only AMD64 images. ARM64 support is planned in the future.

## Build docker image from source

It is also possible to build docker images on your system as follows:

```
mkdir /tmp/glim_docker && cd /tmp/glim_docker
git clone https://github.com/koide3/glim --recursive
git clone https://github.com/koide3/glim_ros1

# Without GPU
docker build -f glim_ros1/docker/noetic/Dockerfile --tag glim_ros1 .

# With GPU
docker build -f glim_ros1/docker/noetic_cuda11.2/Dockerfile --tag glim_ros1 .
```