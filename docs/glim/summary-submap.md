# glim SubMapping 模块总结

## 文件位置

```
src/finder_lidar_mapping/glim/
├── include/glim/mapping/
│   ├── sub_map.hpp                      # SubMap 数据结构
│   ├── sub_mapping_base.hpp             # 抽象基类
│   ├── sub_mapping.hpp                  # 完整子图构建（带优化）
│   ├── sub_mapping_passthrough.hpp      # 轻量子图构建（无优化）
│   └── async_sub_mapping.hpp            # 异步线程包装器
└── src/glim/mapping/
    ├── sub_map.cpp
    ├── sub_mapping_base.cpp
    ├── sub_mapping.cpp
    ├── sub_mapping_create.cpp           # 动态库入口
    ├── sub_mapping_passthrough.cpp
    ├── sub_mapping_passthrough_create.cpp
    └── async_sub_mapping.cpp
```

## 类继承关系

```
SubMappingBase（抽象基类）
├── SubMapping             完整子图构建，带因子图优化
└── SubMappingPassthrough   轻量子图构建，无优化

辅助类：
├── SubMap                  子图数据容器
└── AsyncSubMapping         线程安全的异步包装器
```

## SubMap — 子图数据结构

一个 SubMap 包含一段连续轨迹的所有信息：

| 字段 | 说明 |
|------|------|
| `id` | 子图 ID |
| `T_world_origin` | 子图原点在世界系中的位姿 |
| `T_origin_endpoint_L/R` | 左/右端点相对原点的位姿 |
| `frame` | 合并后的点云（所有关键帧融合） |
| `voxelmaps` | 多分辨率体素地图（用于后续配准） |
| `frames` | 优化后的里程计帧 |
| `odom_frames` | 原始里程计帧（优化前） |

关键方法：
- `save(path)` / `load(path)` — 序列化/反序列化到磁盘
- `drop_frame_points()` — 释放点云数据节省内存，保留位姿信息

## SubMapping vs SubMappingPassthrough 对比

|                    | SubMapping               | SubMappingPassthrough        |
|--------------------|--------------------------|------------------------------|
| **优化**           | LM 因子图优化            | 无优化                       |
| **关键帧选择**     | OVERLAP 或 DISPLACEMENT  | 仅 DISPLACEMENT              |
| **帧间因子**       | GICP 配准                | 无                           |
| **关键帧间因子**   | VGICP（全连接）          | 无                           |
| **IMU**            | 支持预积分因子           | 不支持                       |
| **GPU**            | 可选 GPU 加速            | 不支持                       |
| **体素地图**       | 多分辨率（3 层）         | 单一增量地图                 |
| **计算复杂度**     | O(k²)（k=关键帧数）     | O(1) 每帧                   |
| **适用场景**       | 离线/高精度建图          | 实时/低延迟建图              |

## SubMapping — 完整子图构建

### 主流程 `insert_frame()`

```
里程计帧输入
  ├─ 1帧延迟缓冲（用于 IMU 平滑）
  ├─ [若启用IMU] 轨迹平滑优化
  ├─ [若启用GPU] 点云转 GPU 格式
  ├─ 添加位姿变量 X(current) 到因子图
  ├─ 创建帧间因子：
  │   ├─ BetweenFactor（GICP 配准的相对位姿）
  │   └─ ImuFactor（IMU 预积分，可选）
  ├─ 关键帧判定：
  │   ├─ [OVERLAP] 与已有关键帧的体素重叠度 < 阈值 → 插入
  │   └─ [DISPLACEMENT] 平移/旋转超阈值 → 插入
  ├─ [若为新关键帧]
  │   ├─ 用 IMU 轨迹重新去畸变
  │   ├─ 随机降采样
  │   ├─ 创建多分辨率体素地图（3 层）
  │   └─ 与所有已有关键帧建立 VGICP 因子（全连接）
  ├─ 释放旧帧的点云数据节省内存
  └─ [若关键帧数达到 max_num_keyframes]
     └─ create_submap()
```

### 子图创建 `create_submap()`

```
1. LM 优化因子图 → 得到所有帧的优化位姿
2. 选取中间帧作为子图原点
3. 将所有关键帧点云变换到原点坐标系并合并
4. 降采样（若设置了 target_num_points）
5. 创建多分辨率体素地图
6. 封装为 SubMap 对象输出
```

### 因子图结构

```
X(0) ——BetweenFactor—— X(1) ——BetweenFactor—— X(2) —— ...
 |                       |                       |
PriorFactor          ImuFactor               ImuFactor

关键帧之间：全连接 IntegratedVGICPFactor
  KF0 ←→ KF1
  KF0 ←→ KF2
  KF1 ←→ KF2
  ...
```

- **帧间**：GICP 配准的 BetweenFactor + IMU 预积分因子
- **关键帧间**：VGICP 配准误差因子（全连接，O(k²)）
- **首帧**：PriorFactor 锁定初始位姿

## SubMappingPassthrough — 轻量子图构建

### 主流程

```
里程计帧输入
  ├─ 保存帧元数据（不保留点云）
  ├─ 关键帧判定（仅位移/旋转阈值）
  ├─ [若为新关键帧]
  │   └─ 变换点云到世界系 → 插入增量体素地图
  ├─ 记录体素数量历史
  └─ [触发条件满足]
     └─ create_submap()
```

### 子图创建触发条件

三个条件满足任一即创建：
1. 关键帧数 ≥ `max_num_keyframes`（默认 50）
2. 体素数 ≥ `max_num_voxels`
3. 自适应：体素增长率下降（`adaptive_max_num_voxels`）

创建时直接从体素地图提取点云，**不做任何优化**，原始里程计位姿直接作为最终位姿。

## AsyncSubMapping — 异步包装器

将任意 `SubMappingBase` 实现包装为异步执行：

```
主线程                          后台线程
  │                               │
  ├─ insert_frame() ──→ input_frame_queue ──→ sub_mapping->insert_frame()
  ├─ insert_imu()   ──→ input_imu_queue   ──→ sub_mapping->insert_imu()
  │                               │
  ├─ get_results()  ←── output_submap_queue ←── sub_mapping->get_submaps()
  │                               │
  └─ join()         ──→ end_of_sequence ──→ submit_end_of_sequence() → 退出
```

- 所有输入通过 `ConcurrentVector` 线程安全队列
- 后台线程空闲时 sleep 10ms
- `workload()` 返回输入队列大小，用于负载均衡

## 关键设计要点

### 多分辨率体素地图

SubMapping 为每个关键帧创建 3 层体素地图：
- Level 0: `resolution`
- Level 1: `resolution × scaling_factor`
- Level 2: `resolution × scaling_factor²`

用于粗到细的 VGICP 配准。

### 内存管理

- 非关键帧的点云数据用 `clone_wo_points()` 丢弃
- 旧帧通过 `drop_frame_points()` 释放
- 只有关键帧保留完整点云用于子图合并

### 动态模块加载

通过 `*_create.cpp` 导出 C 接口，支持运行时动态加载：
```cpp
extern "C" SubMappingBase* create_sub_mapping_module() {
    return new SubMapping(SubMappingParams());
}
```
