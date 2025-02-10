# Demo data

See more videos at [Video Gallery](https://github.com/koide3/glim/wiki/Video-Gallery).

## Mapping with various range sensors

- [Download dataset](https://zenodo.org/record/6864654)
- [config_versatile.tar.gz](https://staff.aist.go.jp/k.koide/projects/glim_params/config_versatile.tar.gz)

```bash
ros2 run glim_ros glim_rosbag --ros-args -p config_path:=$(realpath config/kinect) kinect
```

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/rLqYo42eDTQ" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

!!! tip
    **The same parameter set** was used for all the sensors.

!!! warning
    As can be seen in the video, the quality of point clouds of stereo-based sensors (D455 and ZED2i) is not very good, and GLIM, which is based on point cloud matching, does not always work well with these sensors. We recommend using other vision-based SLAM packages for stereo sensors.

## Flat wall experiment

- [Download dataset](https://zenodo.org/records/7641866)  
- [config_flatwall.tar.gz](https://staff.aist.go.jp/k.koide/projects/glim_params/config_flatwall.tar.gz)

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/ouo8pQv4J24" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>


## Outdoor driving test with Livox MID360

- [Download dataset](https://zenodo.org/records/14841855)

!!! tip
    Change parameters in `config_ros.json` as follows (see also the [sensor setup guide](https://github.com/koide3/glim/wiki/Sensor-setup-guide)):
    
    - "acc_scale": 9.80665,
    - "imu_topic": "/livox/imu",
    - "points_topic": "/livox/lidar",

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/CIfRqeV0irE?si=r0BNpCfOh-Aaoek5" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>


## Indoor mapping with Azure Kinect

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/_SI0KR5D8wE?si=W95jKn8vhF28gzyL" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>


## Real-time mapping on Jetson Nano

- [os1_128_01_downsampled.bag (515MB)](https://zenodo.org/record/6859242)
- config_nano_cpu.zip
- config_nano_gpu.zip

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/UxxvB006lrA" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

!!! note
    - Only odometry estimation was performed, no global optimization.
    - Visualization was run on another PC that received points and pose messages via ethernet.  
      (rviz took about a half of Jetson Nano's computation capability without rendering anything!!)
    - (2024/07/04) The current version of GLIM does not support CUDA 11 and older. Some minor midifications are expected to be necessary.
