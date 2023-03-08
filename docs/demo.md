# Demo data

## Mapping with various range sensors

- [Download dataset](https://zenodo.org/record/6864654)
- [config_versatile.zip](https://staff.aist.go.jp/k.koide/projects/glim/config_versatile.zip)

```bash
rosrun glim_ros glim_rosbag _config_path:=$(realpath config/kinect) kinect.bag
```

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/rLqYo42eDTQ" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

!!! tip
    We used **the same parameter set** for all the sensors.

!!! warning
    As can be seen in the video, the quality of point clouds of stereo-based sensors (D455 and ZED2i) is not very good, and GLIM that is based on point cloud matching do not always work well with these sensors. We recommend using other vision-based SLAM packages for stereo sensors.

## Flat wall experiment

- flatwall_dataset.zip (coming soon)  
- [config_flatwall.zip](https://staff.aist.go.jp/k.koide/projects/glim/config_flatwall.zip)

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/ouo8pQv4J24" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
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
