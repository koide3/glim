![GLIM](docs/assets/logo2.png "GLIM Logo")

## Introduction

**GLIM** is a versatile and extensible range-based 3D mapping framework.

- ***Accuracy:*** GLIM is based on direct multi-scan registration error minimization on factor graphs that enables to accurately retain the consistency of mapping results. GPU acceleration is supported to maximize the mapping speed and quality.
- ***Easy-to-use:*** GLIM offers an interactive map correction interface that enables the user to manually correct mapping failures and easily refine mapping results.
- ***Versatility:*** As we eliminated sensor-specific processes, GLIM can be applied to any kind of range sensors including:
    - Spinning-type LiDAR (e.g., Velodyne HDL32e and Ouster OS1-32)
    - Non-repetitive scan LiDAR (e.g., Livox Avia and MID360)
    - Solid-state LiDAR (e.g., Intel Realsense L515)
    - RGB-D camera (e.g., Microsoft Azure Kinect)
- ***Extensibility:*** GLIM provides the global callback slot mechanism that allows to access the internal states of the mapping process and insert additional constraints to the factor graph. We also release [glim_ext](https://github.com/koide3/glim_ext) that offers example implementations of several extension functions (e.g., explicit loop detection, LiDAR-Visual-Inertial odometry estimation).

**Related packages:** [gtsam_points](https://github.com/koide3/gtsam_points), [glim](https://github.com/koide3/glim), ~~[glim_ros1](https://github.com/koide3/glim_ros1),~~ [glim_ros2](https://github.com/koide3/glim_ros2), [glim_ext](https://github.com/koide3/glim_ext)

Tested on Ubuntu 22.04 / 24.04 with CUDA 12.2 / 12.6 / 13.1, and NVIDIA Jetson Orin (Jetpack 6.1).

If you find this package useful for your project, please consider leaving a comment [here](https://github.com/koide3/glim/issues/19). It would help the author receive recognition in his organization and keep working on this project.

[![Build](https://github.com/koide3/glim/actions/workflows/build.yml/badge.svg)](https://github.com/koide3/glim/actions/workflows/build.yml)
[![ROS2](https://github.com/koide3/glim_ros2/actions/workflows/build.yml/badge.svg)](https://github.com/koide3/glim_ros2/actions/workflows/build.yml)
[![EXT](https://github.com/koide3/glim_ext/actions/workflows/build.yml/badge.svg)](https://github.com/koide3/glim_ext/actions/workflows/build.yml)

## Updates

- 2026/01/24 : v1.2.0 released. Added Support for both **GTSAM 4.2a9** and **GTSAM 4.3a0**, and **CUDA 13.1**. Added intensity visualization support.
- 2025/06/15 : The base GTSAM version has been changed. Make sure you have rebuilt and installed **GTSAM 4.3a0** and **gtsam_points 1.2.0**.

## 项目目标 - 隧道/井下巡检车 (揭榜挂帅)

- **数据管理**: 采集数据（rosbag/点云/图像视频）相机覆盖涵洞全内壁（壁/顶/底）并配主动照明保证弱光成像；≥5km 连续巡检续航。
- **多传感器融合感知**: 完成涵洞巡检的在线环境感知工作。环境感知包括可通行区域。
- **多传感器融合导航**: 建图完整无盲区，途径点位姿误差（相对于先验地图的全局定位误差）≤ 路径长度的 0.5% 且不超过 2 m；具备异常处置（失联保护、自动回收/返航、避障）。
- **离线缺陷检测**: 离线对录像做实例分割缺陷检测，并将缺陷定位到 3D 彩色地图。缺陷包括结构裂缝、渗水/积水、衬砌剥落与变形等结构健康隐患。生成缺陷报告。
- **电气与软件安全**: 急停/限速/防护、系统自检/故障降级/安全回收、数据完整性与抗干扰能力。
- **恶劣环境**: 20cm 水深；湿滑/弱光/狭长通道/高湿等工况下稳定通行。
- **多传感器融合建图(加分项)**: 支持无先验地图自主探索建图，输出轨迹、里程计与彩色点云地图。
- **工程化与可靠性(创新/加分)**: 模块化设计、快速部署、低成本高可靠、防护与维护便捷。

## 项目成员
- 指导老师：
    - 胡祝华 海南大学，教授
    - 郭若楠 上海众途智行(电子)科技有限公司，技术负责人
    - 赵瑶池 海南大学，副教授
- 成员：
    - 李若晴 海南大学，人工智能专业博士在读
    - 万子元 上海煤科，工程师
    - 薛慧鸿 海南大学，新一代电子信息技术专业研究生在读
    - 陈锋   海南大学，通信工程专业研究生在读
    - 雷增   海南大学，通信工程专业本科在读
    - 唐容   海南大学，人工智能专业本科在读

## Framework
![定位模块图](docs/assets/定位模块图.png "定位模块图")

## Hardware
![宇树g2](docs/assets/宇树g2.png "宇树g2")
![pts_100产品图](docs/assets/pts_100产品图.png "pts_100产品图")
宇树g2 pts-100

## Dependencies
### Mandatory
- [Eigen](https://eigen.tuxfamily.org/index.php)
- [nanoflann](https://github.com/jlblancoc/nanoflann)
- [GTSAM](https://github.com/borglab/gtsam)
- [gtsam_points](https://github.com/koide3/gtsam_points)

### Optional
- [CUDA](https://developer.nvidia.com/cuda-toolkit)
- [OpenCV](https://opencv.org/)
- [OpenMP](https://www.openmp.org/)
- [ROS/ROS2](https://www.ros.org/)
- [Iridescence](https://github.com/koide3/iridescence)

## Gallery

See more at [Video Gallery](https://github.com/koide3/glim/wiki/Video-Gallery).

| Mapping with various range sensors | Outdoor driving test with Livox MID360 |
|---|---|
|[<img width="480" src="https://github.com/user-attachments/assets/95e153cd-1538-4ca6-8dd0-691e920dccd9">](https://www.youtube.com/watch?v=_fwK4awbW18)|[<img width="480" src="https://github.com/user-attachments/assets/6b337369-a32c-4b07-b0e0-b63f6747cdab">](https://www.youtube.com/watch?v=CIfRqeV0irE)|

| Manual loop closing | Merging multiple mapping sessions |
|---|---|
|![Image](https://github.com/user-attachments/assets/0f02950a-6b7b-437c-a100-21d6575f7c93)|![Image](https://github.com/user-attachments/assets/c77cca29-921b-4e1c-9583-2b962ccda2cb)|

| Object segmentation and removal |  |
|---|---|
|![Image](https://github.com/user-attachments/assets/fd1038e7-c33d-44b1-86f9-8e6474c04210)| |

## Estimation modules

GLIM provides several estimation modules to cover use scenarios, from robust and accurate mapping with a GPU to lightweight real-time mapping with a low-specification PC like Raspberry Pi.

![modules](docs/assets/module.png)

## Thirdparty works using GLIM

If you are willing to add your work here, feel free to let me know in [this thread](https://github.com/koide3/glim/issues/19) :)

- [kamibukuro5656/MapCleaner_Unofficial](https://github.com/kamibukuro5656/MapCleaner_Unofficial)

## License

This package is released under the MIT license. For commercial support, please contact ```k.koide@aist.go.jp```.

If you find this package useful for your project, please consider leaving a comment [here](https://github.com/koide3/glim/issues/19). It would help the author receive recognition in his organization and keep working on this project. Please also cite the following paper if you use this package in your academic work.

## Related work

Koide et al., "GLIM: 3D Range-Inertial Localization and Mapping with GPU-Accelerated Scan Matching Factors", Robotics and Autonomous Systems, 2024, [[DOI]](https://doi.org/10.1016/j.robot.2024.104750) [[Arxiv]](https://arxiv.org/abs/2407.10344)

The GLIM framework involves ideas expanded from the following papers:
- (LiDAR-IMU odometry and mapping) "Globally Consistent and Tightly Coupled 3D LiDAR Inertial Mapping", ICRA2022 [[DOI]](https://doi.org/10.1109/ICRA46639.2022.9812385)
- (Global registration error minimization) "Globally Consistent 3D LiDAR Mapping with GPU-accelerated GICP Matching Cost Factors", IEEE RA-L, 2021, [[DOI]](https://doi.org/10.1109/LRA.2021.3113043)
- (GPU-accelerated scan matching) "Voxelized GICP for Fast and Accurate 3D Point Cloud Registration", ICRA2021, [[DOI]](https://doi.org/10.1109/ICRA48506.2021.9560835)

## Contact
[Kenji Koide](https://staff.aist.go.jp/k.koide/), k.koide@aist.go.jp<br>
National Institute of Advanced Industrial Science and Technology (AIST), Japan

