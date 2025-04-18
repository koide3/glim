# Getting started

## Prerequisite

1. Install GLIM on your system following [the installation section](installation.md). Alternatively, you can also use [prebuilt docker images](docker.md).
2. Download [test data](https://zenodo.org/record/7233945).  
    ROS1: [os1_128_01_downsampled.bag (515MB)](https://zenodo.org/record/7233945/files/os1_128_01_downsampled.bag?download=1) or [os1_128_01.bag (7.3GB)](https://zenodo.org/record/7233945/files/os1_128_01.bag?download=1)  
    ROS2: [os1_128_01_downsampled.tar.gz (426MB)](https://zenodo.org/record/7233945/files/os1_128_01_downsampled.tar.gz?download=1) or [os1_128_01.tar.gz (3.2GB)](https://zenodo.org/record/7233945/files/os1_128_01.tar.gz?download=1)

    Alternative links: ROS1 ([downsampled](https://staff.aist.go.jp/k.koide/projects/glim/datasets/os1_128_01_downsampled.bag))  ROS2 ([downsampled](https://staff.aist.go.jp/k.koide/projects/glim/datasets/os1_128_01_downsampled.tar.gz))

3. Confirm that the sensor configuration and ROS topic parameters are set as follows:
```json
glim/config/config.json
  "config_odometry": "config_odometry_gpu.json",
  "config_sub_mapping": "config_sub_mapping_gpu.json",
  "config_global_mapping": "config_global_mapping_gpu.json",
glim/config/config_sensors.json
  "T_lidar_imu": [-0.006, 0.012, -0.008, 0, 0, 0, 1],
glim/config/config_ros.json
  "imu_topic": "/os_cloud_node/imu",
  "points_topic": "/os_cloud_node/points",
```

!!! tip
    If you want to try the **CPU-based odometry estimation and global optimization**, in `config.json`, set   
      `"config_odometry"` : `"config_odometry_cpu.json"`,  
      `"config_sub_mapping"` : `"config_sub_mapping_passthrough.json"`,  
      `"config_global_mapping"` : `"config_global_mapping_pose_graph.json"`,

!!! tip
    If you want to try the **LiDAR-only odometry estimation without IMU data**,  in `config.json`, set  
      `"config_odometry"` : `"config_odometry_ct.json"`,  
    and, in `config_sub_mapping_gpu.json` and `config_global_mapping_gpu.json`, set  
      `"enable_imu"` : `false`

!!! warning
    In ROS2, you need to re-run `colcon build` to apply config changes in the installed packages. See the [configuration files](#configuration-files) section for more details.

## Executables

GLIM provides two ROS executables: ***glim_rosnode*** and ***glim_rosbag***.


### glim_rosnode
***glim_rosnode*** launches GLIM as a standard ROS node that subscribes to points, imu, and image topics. 

<details>
<summary>ROS1 command</summary>

```bash
# Start roscore
roscore
```

```bash
# Enable use_sim_time and launch GLIM as a standard ROS node on another terminal
rosparam set use_sim_time true
rosrun glim_ros glim_rosnode
```

```bash
# Play rosbag on yet another terminal
rosbag play --clock os1_128_01.bag
```

```bash
# Visualize on rviz (optional)
rviz -d glim_ros1/rviz/glim_ros.rviz
```
</details>

<details>
<summary>ROS2 command</summary>

```bash
ros2 run glim_ros glim_rosnode
```

```bash
ros2 bag play os1_128_01
```

```bash
rviz2 -d glim_ros2/rviz/glim_ros.rviz
```
</details>



<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/lM1Qh3PCvCQ" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

### glim_rosbag

***glim_rosbag*** launches a mapping instance that directly reads data from rosbag. It automatically adjusts the playback speed while avoiding data drop to perform mapping in a minimum time.

<details>
<summary>ROS1 command</summary>

```bash
roscore
```

```bash
rosparam set use_sim_time true
```

```bash
rosrun glim_ros glim_rosbag os1_128_01.bag
```
</details>

<details>
<summary>ROS2 command</summary>

```bash
ros2 run glim_ros glim_rosbag os1_128_01
```
</details>



## Configuration files

GLIM reads parameter settings from JSON files in a config root directory, which is set to ```glim/config``` by default. It first reads ```config.json``` that describes relative paths to submodule parameter files, and then reads parameters of submodules from specified configuration files. The config root directory can be changed by setting ```config_path``` ROS param when starting GLIM executables.

!!! note
    If ```config_path``` starts with "/", the path is interpreted as an absolute path. Otherwise, ```config_path``` is interpreted as a path relative to ```glim``` package directory.  
    `realpath` command is useful to run GLIM with local configuration files out of the package directory: (e.g., `ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath config)`)

!!! note
    On ROS2, you need to run ```colcon build``` to apply changes of the configuration files in the package directory because ROS2 requires to place config files in the install directory. To avoid this, use `--symlink-install` option for `colcon build`.

!!! info
    See [Important parameters](parameters.md) to understand parameters that should be fine-tuned.

**Example**

<details>
<summary>ROS1 command</summary>

```bash
# Load parameters from "glim/config/presets/gpu/config.json"
rosrun glim_ros glim_rosnode _config_path:=config/presets/gpu

# Load parameters from "/tmp/config/config.json"
rosrun glim_ros glim_rosnode _config_path:=/tmp/config

# Load parameters from "./config/config.json"
rosrun glim_ros glim_rosnode _config_path:=$(realpath ./config)
```
</details>

<details>
<summary>ROS2 command</summary>

```bash
# Load parameters from "glim/config/presets/gpu/config.json"
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=config/presets/gpu

# Load parameters from "/tmp/config/config.json"
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/tmp/config

# Load parameters from "./config/config.json"
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ./config)
```
</details>

## Mapping result

The mapping result data (dump data) will be saved in ```/tmp/dump``` when closing glim_rosnode or glim_rosbag. The dump data can be visualized and edited using the offline viewer (```rosrun glim_ros offline_viewer```).

The dump directory contains mapping graph data as well as the following estimated trajectory files.

Estimated trajectory files (TUM format : Each row is [t x y z qx qy qz qw]):

  - odom_imu.txt : Trajectory of the IMU frame without loop closure (Odometry result)
  - traj_imu.txt : Trajectory of the IMU frame with loop closure (Global mapping result)
  - odom_lidar.txt : Trajectory of the LiDAR frame without loop closure (Odometry result)
  - traj_lidar.txt : Trajectory of the LiDAR frame with loop closure (Global mapping result)

**Example dump data**: [dump_rosbag2_2024_04_16-14_17_01.tar.gz](https://staff.aist.go.jp/k.koide/projects/glim/datasets/dump_rosbag2_2024_04_16-14_17_01.tar.gz) (trajectory errors are injected for manual loop closure test)

### Offline viewer (manual map editing and point cloud export)

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/YUbiNTa36cc?si=G95A6sReF-_GiqYz" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>

```bash
# ROS1
rosrun glim_ros offline_viewer

# ROS2
ros2 run glim_ros offline_viewer
```

#### Open map
```File``` -> ```Open Map``` -> Select a dump directory.

#### Create explicit loop constraints
- ```Right click a submap sphere``` -> ```Loop begin``` -> ```Right click another submap sphere``` -> ```Loop end```
- Roughly align red and green point clouds -> Press ```Align``` to perform scan matching -> Press ```Create Factor``` if the alignment result is fine.

#### Create Plane-BA constraints
- ```Right click a point on a flat surface``` -> ```Bundle Adjustment (Plane)```
- Adjust the sphere size so it covers sufficient points on the plane -> ```Create Factor```

#### Export map point cloud (PLY format)
- ```File``` -> ```Save``` -> ```Export Points```

### ... and more

- [Merging multiple mapping sessions (`offline_viewer`)](merge.md)
- [Manual points selection and removal (`map_editor`)](edit.md)

## Setup your own sensor

See [Sensor setup guide](https://github.com/koide3/glim/wiki/Sensor-setup-guide).
