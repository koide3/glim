<!--
 * @Author: meitiever
 * @Date: 2026-06-05 19:33:47
 * @LastEditors: meitiever
 * @LastEditTime: 2026-06-05 19:40:36
 * @Description: content
-->
# datasets

隧道 / 井下巡检车（隧道·井下巡检车，揭榜挂帅项目）采集的多传感器数据集，用于 [GLIM](https://github.com/meitiever82/glim_underground) 建图与定位的开发与验证。

## 下载

百度网盘：

> 链接: https://pan.baidu.com/s/1wHw-VfWapnTMH3mzVKyNxw 提取码: `5w5d`

## 采集设备

- **LiDAR**：RoboSense Airy（含内置 IMU）
- **采集场景**：涵洞 / 隧道 / 井下狭长通道（弱光、潮湿、积水等工况）
- **录制格式**：ROS2 `rosbag2`（`sqlite3` `.db3` 或 `.mcap`）

## 数据内容

| 话题 | 类型 | 说明 |
|---|---|---|
| `/rslidar_points` | `sensor_msgs/msg/PointCloud2` | LiDAR 点云（含 `time` / `intensity` 字段） |
| `/rslidar_imu_data` | `sensor_msgs/msg/Imu` | LiDAR 内置 IMU 数据 |

> 注：实际话题名以各 bag 内 `metadata.yaml` 为准。可用 `ros2 bag info <bag_dir>` 查看时长、话题与消息数。

## 目录结构

```
datasets/
└── <bag_name>/
    ├── metadata.yaml
    └── <bag_name>_0.db3        # 或 .mcap
```

## 使用方法

1. 下载并解压数据到本地目录。
2. 启动 GLIM 建图（话题名 / TF / 外参可在 `config/casbot_mapping` 中调整）：

```bash
source install/setup.bash
ros2 launch glim_ros2 glim_robosense_airy.launch.py
```

3. 回放数据：

```bash
ros2 bag play <bag_dir>
```

若 bag 中的话题与 launch 默认值（`/rslidar_points`、`/rslidar_imu_data`）不一致，可通过 launch 参数覆盖：

```bash
ros2 launch glim_ros2 glim_robosense_airy.launch.py \
    points_topic:=/your/points imu_topic:=/your/imu
```

## 相关仓库

- [glim_underground](https://github.com/meitiever82/glim_underground) — GLIM 建图核心（fork 自 koide3/glim）
- [glim_ros2](https://github.com/meitiever82/glim_ros2) — ROS2 节点与 launch
- [glim_ext](https://github.com/meitiever82/glim_ext) — 扩展模块
