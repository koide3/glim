# glim Odometry Estimation 模块总结

## 文件位置

```
src/finder_lidar_mapping/glim/
├── include/glim/odometry/
│   ├── odometry_estimation_base.hpp     # 抽象基类
│   ├── odometry_estimation_ct.hpp
│   ├── odometry_estimation_imu.hpp
│   ├── odometry_estimation_cpu.hpp
│   ├── odometry_estimation_gpu.hpp
│   ├── async_odometry_estimation.hpp    # 异步线程包装器
│   ├── callbacks.hpp                    # 回调槽定义
│   └── estimation_frame.hpp             # 帧数据结构
└── src/glim/odometry/
    ├── odometry_estimation_ct.cpp
    ├── odometry_estimation_imu.cpp
    ├── odometry_estimation_cpu.cpp
    ├── odometry_estimation_gpu.cpp
    └── async_odometry_estimation.cpp

src/finder_lidar_mapping/glim_ext/modules/odometry/
├── orb_slam_frontend/       # ORB-SLAM3 视觉前端
├── gravity_estimator/       # 重力方向估计
├── imu_validator/           # IMU 标定质量验证
├── uncertainty_estimator/   # 位姿不确定性分析
└── velocity_suppressor/     # 速度抑制约束
```

## 类继承关系

```
OdometryEstimationBase
├── OdometryEstimationCT              纯LiDAR，独立实现
└── OdometryEstimationIMU             IMU基类，模板方法模式
    ├── OdometryEstimationCPU         CPU版配准（GICP / VGICP）
    └── OdometryEstimationGPU         GPU版配准（VGICP on CUDA）
```

## 各类对比

|                    | CT                         | CPU                          | GPU                          |
|--------------------|----------------------------|------------------------------|------------------------------|
| **IMU**            | 不用                       | 用                           | 用                           |
| **配准算法**       | CT-GICP（连续时间）        | GICP 或 VGICP               | VGICP (CUDA)                 |
| **配准方式**       | scan-to-model (iVox)       | scan-to-model (iVox/VoxelMap)| scan-to-keyframes            |
| **去畸变**         | 配准中隐式完成（CT插值）   | 配准前由IMU预积分完成        | 配准前由IMU预积分完成        |
| **坐标系**         | LiDAR 系                   | IMU 系                       | IMU 系                       |
| **每帧优化变量**   | X(i), Y(i)                 | X(i), V(i), B(i)            | X(i), V(i), B(i)            |
| **位姿预测**       | 恒速模型（0.85衰减）       | IMU 预积分                   | IMU 预积分                   |
| **后端优化**       | LM局部 + iSAM2滑窗         | LM局部 + iSAM2滑窗          | iSAM2滑窗（无单独LM）       |
| **目标地图/关键帧**| 单一增量iVox地图           | 单一增量地图                 | 多关键帧集合                 |

## 各类详解

### OdometryEstimationCT — 纯LiDAR连续时间里程计

核心思路：用 CT-GICP 将当前帧与增量体素地图配准，估计扫描起止两端的位姿。

**流程：**

1. 用恒速模型（前两帧位姿差 × 0.85衰减）预测当前帧起止位姿
2. 构建局部因子图：CT-GICP因子 + 位置一致性先验 + 恒速约束
3. LM 优化求解 `X(i)`, `Y(i)`（扫描起点/终点位姿）
4. CT-GICP 内部对每个点按时间插值位姿，隐式完成去畸变
5. 去畸变后的点云变换到世界系插入 iVox 地图
6. 将 LM 结果以强先验形式喂给 iSAM2 滑窗做全局平滑

**关键设计：**

- 双变量 `X(i)`/`Y(i)` 表示扫描起止，支持连续时间建模
- 两层优化：LM 做精确配准 → iSAM2 做滑窗平滑
- iVox 支持增量插入 + LRU 淘汰，维持局部地图大小可控

---

### OdometryEstimationIMU — IMU基类（模板方法模式）

负责所有与 IMU 相关的通用逻辑，子类只需实现 `create_factors()` 提供配准因子。

**首帧初始化：**

- NAIVE 模式：直接用配置的初始状态
- LOOSE 模式：用多帧 IMU 数据估计重力方向，自动对齐世界系

**后续帧流程：**

1. **帧间 IMU 预积分**（inter-scan）：上一帧 → 当前帧，预测 `T_world_imu`、`v_world_imu`，生成 `ImuFactor`
2. **帧内 IMU 预积分去畸变**（intra-scan）：扫描起始 → 结束，插值 IMU 轨迹，对每个点 deskew
3. 协方差估计 → 调用子类 `create_frame()`（GPU版建体素地图）
4. 调用子类 `create_factors()`（子类提供配准因子）
5. iSAM2 滑窗更新（额外一轮 update）
6. 边缘化超出窗口的旧帧，回写优化后的位姿/速度/bias

**IMU bias 处理：**

- `BetweenFactor` 约束相邻帧 bias 缓慢变化（`sigma = imu_bias_noise`）
- 可选 `fix_imu_bias` 用先验锁定 bias 不变

---

### OdometryEstimationCPU — CPU版配准

继承 IMU 基类，实现 `create_factors()` 提供 scan-to-model 配准因子。

**两种配准模式：**

- **GICP 模式**：用 iVox 做近邻搜索，`IntegratedGICPFactor` 计算残差（点到点+协方差）
- **VGICP 模式**：用多层 `GaussianVoxelMapCPU`，点到高斯分布配准，无需显式找对应点

**配准流程：**

1. 在 target 坐标系中用 LM 优化 `X(current)` 与目标地图配准
2. 自定义收敛判定：平移增量 < 1e-3m 且旋转增量 < 1e-3°
3. 将优化结果转换为帧间相对位姿，以 `BetweenFactor` 形式传给 iSAM2

**目标地图更新：**

- 第 5 帧之后随机降采样至 10%（`target_downsampling_rate`）后再插入，控制地图增长
- iVox / VoxelMap 均有 LRU 淘汰机制

---

### OdometryEstimationGPU — GPU版配准

继承 IMU 基类，实现 `create_factors()` 提供 **scan-to-keyframes** 配准因子。

**与 CPU 版最大区别：不维护增量地图，而是维护一组关键帧。**

**配准因子构建：**

- **近邻帧**（`full_connection_window_size`，默认3）：二元 VGICP 因子，两端位姿都优化
- **关键帧**：
  - 在滑窗内 → 二元因子（两端都优化）
  - 已被边缘化 → 一元因子（关键帧位姿固定）
- 每个因子通过 `IntegratedVGICPFactorGPU` 在 CUDA 上并行计算
- `StreamTempBufferRoundRobin` 实现多 CUDA stream 并行

**自适应体素分辨率：**

- 根据点云中位距离在 `[voxel_resolution, voxel_resolution_max]` 之间线性插值
- 近处场景用小分辨率（细节多），远处用大分辨率（点稀疏）

**三种关键帧管理策略：**

| 策略         | 来源                    | 插入条件                         | 淘汰逻辑                                               |
|--------------|-------------------------|----------------------------------|---------------------------------------------------------|
| **OVERLAP**  | Koide, ICRA2022         | 与所有关键帧重叠度 < max_overlap | 先删无重叠的；再按 `overlap_latest × (1-overlap_others)` 打分删最低 |
| **DISPLACEMENT** | Engel (DSO), PAMI2018 | 位移或旋转超阈值                 | 先删无重叠的；再按 `√d_new × Σ(1/d_others)` 打分删最高（离新帧远且扎堆的） |
| **ENTROPY**  | Kuo, ICRA2020           | 负熵低于滑动平均 × 阈值          | 直接删最老的                                             |

## 图像输入管线

里程计系统通过回调机制支持图像观测，但核心类本身不做视觉处理，而是将图像转发给扩展模块。

### 图像数据流

```
外部输入 image
  ↓
AsyncOdometryEstimation::insert_image()
  → input_image_queue（线程安全队列）
  → 与 IMU 时间戳同步后转发
  ↓
OdometryEstimationBase::insert_image()
  → OdometryEstimationCallbacks::on_insert_image（回调槽）
  ↓                                          ↓
OdometryEstimationGPU::insert_image()     扩展模块（如 ORB-SLAM）
  → image_buffer（最多缓存10帧）            → 视觉处理 → 生成因子
  → TODO: LVIO 功能尚未实现                → on_smoother_update 注入因子图
```

- `OdometryEstimationGPU` 重写了 `insert_image()`，缓存图像准备做 LVIO，但目前只有 TODO
- `OdometryEstimationCPU` 和 `OdometryEstimationCT` 不处理图像，仅触发回调
- `EstimationFrame` 已预留相机相关字段：`T_cam_imu`、`T_world_cam`、`FrameID::CAMERA`

## 回调机制与扩展模块

### 回调槽 (`OdometryEstimationCallbacks`)

扩展模块通过 `CallbackSlot` 注册回调，在里程计的关键节点被调用：

| 回调 | 触发时机 | 扩展模块用途 |
|------|----------|-------------|
| `on_insert_imu` | 收到 IMU 数据 | 重力估计、IMU 验证 |
| `on_insert_image` | 收到图像 | ORB-SLAM 视觉前端 |
| `on_insert_frame` | 收到点云帧 | — |
| `on_new_frame` | 帧预处理完成（IMU 预测后） | 不确定性估计、速度抑制 |
| `on_update_new_frame` | 单帧位姿更新后 | — |
| `on_update_frames` | 滑窗内所有帧位姿更新后 | 重力估计 |
| `on_smoother_update` | iSAM2 优化前 | **注入额外因子的关键入口** |
| `on_smoother_update_finish` | iSAM2 优化后 | — |
| `on_marginalized_frames` | 帧被边缘化 | — |

关键设计：`on_smoother_update` 回调的参数包含 `new_factors` 和 `new_values` 的引用，扩展模块可以直接向其中添加因子，在本轮优化中一起求解。

### 扩展模块架构

所有扩展模块继承 `ExtensionModule`，通过动态库加载：

```cpp
class ExtensionModule {
    virtual bool needs_wait() const;   // 是否需要等待此模块
    virtual bool ok() const;           // 模块是否健康
    virtual void at_exit(path);        // 退出时清理
    static shared_ptr<ExtensionModule> load_module(so_name);  // 动态加载
};
```

## 扩展模块详解

### ORB-SLAM Frontend — 视觉里程计前端

**功能**：用 ORB-SLAM3 提供视觉-惯性里程计约束。

**注册回调**：`on_insert_imu`、`on_insert_image`、`on_new_frame`、`on_smoother_update`

**流程**：

```
on_insert_imu  → IMU 队列 ─┐
on_insert_image → 图像队列 ─┤
                            ↓
                frontend_task()（后台线程）
                  ├─ 灰度化 + CLAHE 直方图均衡
                  ├─ 收集帧间 IMU 数据
                  ├─ ORB-SLAM3 TrackMonocular()
                  └─ 输出 T_base_camera 位姿序列
                            ↓
                create_vio_factors()
                  ├─ 与里程计帧时间对齐（SLERP 插值）
                  ├─ 计算 VIO 相对位姿 vs 里程计相对位姿
                  └─ 生成 BetweenFactor<Pose3>
                            ↓
on_smoother_update → 注入 VIO 因子到 iSAM2
```

**因子精度**：平移 1e6，旋转 1e3（对视觉约束有高置信度）。

> 注意：CMakeLists 标注 "This module is not maintained"。

---

### Gravity Estimator — 重力方向估计

**功能**：独立估计重力方向，约束里程计的俯仰/翻滚不漂移。

**注册回调**：`on_insert_imu`、`on_update_frames`、`on_smoother_update`

**工作方式**：

- 内部维护独立的 `IncrementalFixedLagSmoother`，用 IMU 因子 + 位姿因子独立优化
- 估计出重力对齐的位姿后，生成 `GravityAlignmentFactor`（约束位姿的 Z 轴方向与重力一致）
- 第 100 帧后追加 IMU bias 强先验（sigma=1e-3），锁定 bias 估计
- 通过 `on_smoother_update` 回调注入主里程计因子图

---

### IMU Validator — IMU 标定验证

**功能**：比较 LiDAR 推算的角速度与 IMU 测量值，验证标定质量。

**注册回调**：`on_insert_imu`、`on_new_frame`

**输出指标**：

| 指标 | 含义 | 阈值 |
|------|------|------|
| NID（归一化信息距离） | LiDAR 与 IMU 角速度一致性 | < 0.2 好，< 0.3 警告，≥ 0.3 差 |
| IMU 时间偏移 | 搜索 [-0.5s, +0.5s] 的最佳对齐 | < 0.02s 好，< 0.05s 警告 |

**不注入因子**，仅分析和发布诊断信息。

---

### Uncertainty Estimator — 位姿不确定性分析

**功能**：从 iSAM2 提取位姿协方差，检测退化状态。

**注册回调**：`on_new_frame`

**分析方式**：

- 从 `EstimationFrame` 的 `custom_data["pose_covariance"]` 取 6×6 协方差
- 分别对平移（右下 3×3）和旋转（左上 3×3）做特征值分解
- 逆条件数 = √(λ_min / λ_max)，接近 0 表示退化

**退化判定**：平移或旋转的逆条件数 < 0.1 → 标记为退化状态。

**不注入因子**，仅发布诊断信息。

---

### Velocity Suppressor — 速度抑制

**功能**：对低速或静止平台，用软约束将速度拉向零。

**注册回调**：`on_new_frame`、`on_smoother_update`

**注入因子**：

1. **静止先验**：`PriorFactor<Vector3>(V(i), 零, sigma=stationary_prior_inf_scale)` — 弱约束拉向零
2. **鲁棒速度约束**：`PriorFactor` + `L2WithDeadZone` 鲁棒核
   - 速度 < `max_velocity` 时无惩罚（死区）
   - 超出后二次增长但有鲁棒下压
   - 防止速度估计飘飞，同时不压制合法运动

## 公共设计模式

### iSAM2 固定滞后平滑

四个核心类都使用 `IncrementalFixedLagSmootherExtWithFallback`：
- 滑窗时间 `smoother_lag`（默认 5 秒）
- 超出窗口的帧被边缘化释放内存
- Fallback 机制防止优化器发散

### TBB 线程控制

所有类在 `GTSAM_USE_TBB` 下通过 `tbb::task_arena` 限制优化器并行线程数，防止 TBB 线程与其他模块冲突。

### AsyncOdometryEstimation — 异步包装器

将任意 `OdometryEstimationBase` 包装为后台线程执行：

```
主线程                              后台线程
  ├─ insert_frame() ──→ 队列 ──→ odometry->insert_frame()
  ├─ insert_imu()   ──→ 队列 ──→ odometry->insert_imu()
  ├─ insert_image()  ──→ 队列 ──→ 与 IMU 时间同步后转发 odometry->insert_image()
  └─ get_results()  ←── 输出队列 ←── 处理结果
```

图像队列有特殊的时间同步逻辑：等待 IMU 数据到达后再转发图像，保证时序一致性。
