# Extension modules

## Open-source extension modules

[glim_ext](https://github.com/koide3/glim_ext) provides example implementations of extension modules that demonstrate the extensibility of GLIM. All the implemented modules are decoupled from GLIM main components and inter-module communication is conducted via the [global callback slot mechanism](extend.md).

!!! warning
    The implementations in **glim_ext** are proof-of-concept code for showing the extensibility of GLIM. They may not be well-maintained and may not be suitable for practical purposes.

!!! warning
    Each module in glim_ext uses several external libraries that employ different licenses. **You must carefully check and follow their licensing conditions**.

### Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/koide3/glim_ext

cd ~/ros2_ws
colcon build
```

### Odometry estimation modules

#### [ORB_SLAM-based loose visual integration](https://github.com/koide3/glim_ext/tree/master/modules/odometry/orb_slam_frontend)

- Loosely coupled visual odometry estimation constraints based on ORB_SLAM3
- Dependency: [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) (GPL-3.0)

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/-AGdYyd0ZNQ" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>


#### [Velocity suppressor](https://github.com/koide3/glim_ext/tree/master/modules/odometry/velocity_suppressor)

- Constraints to regulate IMU velocity

#### [IMU calibration validator](https://github.com/koide3/glim_ext/tree/master/modules/odometry/imu_validator)

- Utility module to validate the LiDAR-IMU transformation (See [FAQ](faq.md))

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/tsOJHTObuqY" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

### Global optimization modules

#### [GNSS constraints](https://github.com/koide3/glim_ext/tree/master/modules/mapping/gnss_global) [ROS2 only]

- Naive implementation of GNSS global optimization constraints

#### [ScanContext loop detector](https://github.com/koide3/glim_ext/tree/master/modules/mapping/scan_context_loop_detector)

- Explicit loop detection based on ScanContext
- Dependency: [ScanContext](https://github.com/irapkaist/scancontext) (CC BY-NC-SA 4.0)

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/7Pdffxhfg4o" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

#### [DBoW loop detector](https://github.com/koide3/glim_ext/tree/master/modules/mapping/dbow_loop_detector)

- Explicit loop detection based on DBoW3
- Dependency: [DBoW3](https://github.com/rmsalinas/DBow3) ([LICENSE](https://github.com/rmsalinas/DBow3/blob/master/LICENSE.txt))

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/wMdiIcSE5qg" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

## Closed-source extension modules

The following modules are provided as closed-source packages. Contact us if you are interested in the closed-source modules.

### Tightly-coupled multi-LiDAR odometry estimation module

### Tightly-coupled multi-camera odometry estimation module

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/LBogW_6Appg" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

### Dense mapping and colorization module

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/PpKKs7S1niM" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>
