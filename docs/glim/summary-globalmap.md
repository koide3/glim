# glim GlobalMapping 模块总结

## 文件位置

```
src/finder_lidar_mapping/glim/
├── include/glim/mapping/
│   ├── global_mapping_base.hpp          # 抽象基类
│   ├── global_mapping.hpp               # 现代实现（推荐）
│   ├── global_mapping_pose_graph.hpp    # 传统位姿图实现（遗留）
│   └── async_global_mapping.hpp         # 异步线程包装器
└── src/glim/mapping/
    ├── global_mapping_base.cpp
    ├── global_mapping.cpp
    ├── global_mapping_create.cpp
    ├── global_mapping_pose_graph.cpp
    ├── global_mapping_pose_graph_create.cpp
    └── async_global_mapping.cpp
```

## 类继承关系

```
GlobalMappingBase（抽象基类）
├── GlobalMapping            现代实现，隐式回环 + 点云匹配因子（推荐）
└── GlobalMappingPoseGraph   传统实现，显式回环检测 + BetweenFactor（遗留）

辅助类：
└── AsyncGlobalMapping       线程安全的异步包装器
```

## GlobalMapping vs GlobalMappingPoseGraph 对比

|                    | GlobalMapping              | GlobalMappingPoseGraph       |
|--------------------|----------------------------|------------------------------|
| **推荐程度**       | 推荐使用                   | 遗留代码，不推荐             |
| **回环检测**       | 隐式（插入时自动检测重叠） | 显式（后台线程 GICP 配准）   |
| **匹配因子**       | IntegratedVGICPFactor      | BetweenFactor                |
| **IMU**            | 支持预积分因子             | 不支持                       |
| **GPU**            | 支持 GPU 加速              | 仅 CPU                       |
| **体素地图**       | 多分辨率自适应             | 单分辨率                     |
| **回环因子精度**   | 高（持续优化匹配残差）     | 低（固定相对位姿约束）       |

## GlobalMapping — 现代实现

### 核心思路

每插入一个子图，自动搜索所有空间上重叠的历史子图，建立 **VGICP 匹配代价因子**（而非简单的 BetweenFactor），然后用 ISAM2 增量优化。

### `insert_submap()` 主流程

```
新子图输入
  │
  ├─ 1. 初始化位姿（从里程计或前一子图推算）
  │
  ├─ 2. 创建多分辨率体素地图（自适应分辨率）
  │     ├─ 计算点云中位距离
  │     ├─ 线性插值确定基础分辨率
  │     └─ 创建 N 层体素地图（粗到细）
  │
  ├─ 3. create_between_factors()
  │     └─ 与前一子图建立里程计约束
  │         ├─ [GICP] LM 优化配准 → BetweenFactor + 信息矩阵
  │         └─ [NONE] 直接用里程计相对位姿
  │
  ├─ 4. create_matching_cost_factors()
  │     ├─ 搜索 max_implicit_loop_distance 内的所有子图
  │     ├─ 计算体素重叠度
  │     ├─ 重叠 ≥ min_implicit_loop_overlap → 创建 IntegratedVGICPFactor
  │     └─ 若与前一子图重叠不足 → 补充 BetweenFactor 保证连通性
  │
  ├─ 5. [若启用IMU] 创建 IMU 因子
  │     ├─ ImuFactor（预积分）
  │     ├─ BetweenFactor<Pose3>（端点约束）
  │     ├─ RotateVector3Factor（速度旋转约束）
  │     └─ PriorFactor<ImuBias>（偏置先验）
  │
  ├─ 6. update_isam2()
  │     ├─ 增量更新 ISAM2
  │     └─ [异常处理] IndeterminantLinearSystemException
  │         → 插入 LinearDampingFactor → 重置 ISAM2 → 重试
  │
  └─ 7. 更新所有子图的优化位姿
```

### 隐式回环检测

与传统的"检测回环 → 添加约束"不同，GlobalMapping 采用隐式方式：

```
对每个新子图：
  遍历所有历史子图 i：
    if 距离(i, current) < max_implicit_loop_distance:
      overlap = 计算体素重叠度(i, current)
      if overlap ≥ min_implicit_loop_overlap:
        添加 IntegratedVGICPFactor(X(i), X(current))
```

- **不需要单独的回环检测模块**，配准因子在优化过程中持续约束相对位姿
- 比 BetweenFactor 更准确：每次 ISAM2 重线性化时，匹配残差会重新计算
- 代价是因子数量多，但 GPU 加速可以缓解

### `find_overlapping_submaps()` — 手动触发回环搜索

可在任意时刻调用，搜索所有子图对的重叠关系并添加新因子。适用于：
- 后处理阶段补充回环
- 位姿更新后发现新的重叠

### ISAM2 错误恢复

```
update_isam2()
  try:
    isam2->update(factors, values)
  catch IndeterminantLinearSystemException:
    找到问题变量
    插入 LinearDampingFactor（6D, scale=1e3）
    重置 ISAM2
    递归重试
```

防止因退化几何导致优化器发散。

### 自适应体素分辨率

```
dist_median = 点云中位距离
p = clamp((dist_median - dmin) / (dmax - dmin), 0, 1)
base_resolution = voxel_resolution + p * (voxel_resolution_max - voxel_resolution)

Level 0: base_resolution
Level 1: base_resolution × scaling_factor
Level 2: base_resolution × scaling_factor²
```

近处场景用小分辨率（细节多），远处用大分辨率（点稀疏）。

## GlobalMappingPoseGraph — 传统实现

> 源码注释："We recommend using GlobalMapping instead of this class if accuracy matters."

### 核心思路

传统两阶段方式：前端检测回环候选 → 后台线程 GICP 验证 → 添加 BetweenFactor。

### 主流程

```
新子图输入
  │
  ├─ 创建 SubMapTarget（点云降采样 + KdTree/VoxelMap）
  ├─ 计算累积行驶距离 travel_dist
  ├─ 创建里程计 BetweenFactor
  │
  ├─ find_loop_candidates()
  │   ├─ 搜索空间上近但行驶距离远的子图对
  │   │   （距离 < max_neighbor_dist 且 travel_dist > min_travel_dist）
  │   └─ 生成 LoopCandidate 放入队列
  │
  └─ update ISAM2

后台线程 loop_detection_task()（持续运行）：
  ├─ 从队列取 LoopCandidate
  ├─ GICP/VGICP 配准（LM 优化）
  ├─ 计算误差和内点比例
  ├─ 内点比例 ≥ min_inlier_fraction → 回环有效
  │   └─ 创建 BetweenFactor（带鲁棒核函数）
  └─ 放入 detected_loops 队列

optimize()：
  ├─ 收集所有 detected_loops
  ├─ 添加到 ISAM2
  └─ 更新子图位姿
```

### 与 GlobalMapping 的本质区别

| 方面 | GlobalMapping | GlobalMappingPoseGraph |
|------|---------------|------------------------|
| **回环因子类型** | VGICP 匹配代价因子（每次线性化重新计算残差） | BetweenFactor（固定相对位姿，一次性） |
| **回环时机** | 每个子图插入时同步检测 | 后台线程异步检测 |
| **信息利用** | 持续利用点云信息 | 配准后丢弃点云，只保留相对位姿 |

## AsyncGlobalMapping — 异步包装器

将 `GlobalMappingBase` 包装为后台线程执行，避免阻塞前端：

```
主线程                              后台线程
  │                                   │
  ├─ insert_submap() ──→ 队列 ──→ global_mapping->insert_submap()
  ├─ insert_imu()    ──→ 队列 ──→ global_mapping->insert_imu()
  │                                   │
  │                    定时/请求触发：
  │                    ├─ optimize()
  │                    ├─ find_overlapping_submaps()
  │                    └─ recover_graph()
  │                                   │
  ├─ save()          ─── mutex ──→ global_mapping->save()
  └─ join()          ──→ 处理完队列后退出
```

关键设计：
- `optimization_interval`（默认 5 秒）定时触发优化
- 通过 `request_to_optimize` / `request_to_find_overlapping_submaps` 原子标志请求操作
- `global_mapping_mutex` 保护底层实现的线程安全

## 整体数据流

```
里程计帧
  ↓
SubMapping / SubMappingPassthrough
  ↓ 输出 SubMap
AsyncSubMapping（异步）
  ↓
AsyncGlobalMapping（异步）
  ↓
GlobalMapping / GlobalMappingPoseGraph
  ├─ 子图间配准/回环检测
  ├─ ISAM2 增量优化
  └─ 输出全局优化后的子图位姿
  ↓
save() → 磁盘持久化
export_points() → 全局点云导出
```

## 关键设计要点

### 因子图变量

| 符号 | 含义 | 使用条件 |
|------|------|----------|
| `X(i)` | 第 i 个子图的位姿 | 始终 |
| `V(i)` | 第 i 个子图端点的速度 | 启用 IMU |
| `B(i)` | 第 i 个子图的 IMU bias | 启用 IMU |

### 动态模块加载

通过 `*_create.cpp` 导出 C 接口，运行时可动态切换实现：
```cpp
extern "C" GlobalMappingBase* create_global_mapping_module() {
    return new GlobalMapping(GlobalMappingParams());
}
```

### 图恢复机制 `recover_graph()`

用于加载保存的地图时修复可能的图损坏：
- 检查缺失的变量（X, V, B）
- 检查连通性，补充缺失因子
- 处理多 session 映射的 key 重映射
